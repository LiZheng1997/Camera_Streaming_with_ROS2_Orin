#!/usr/bin/env python3
import asyncio
import json
import ssl
import time
import websockets
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import threading


class CmdVelReceiverSecure(Node):
    def __init__(self):
        super().__init__('cmdvel_receiver_secure')

        self.target_uri = "wss://139.180.169.115:9443"
        self.auth_token = "secure_token_12345"
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

        self.last_heartbeat = time.time()
        self.reconnect_delay = 3.0
        self.running = True

        self.get_logger().info(f"🚗 Vehicle secure client initialized. Target: {self.target_uri}")

        # 在独立线程中启动完整的 asyncio 环境
        self.thread = threading.Thread(target=self._run_async_loop, daemon=True)
        self.thread.start()

    def _run_async_loop(self):
        """在独立线程中创建并运行完整的事件循环"""
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        
        try:
            loop.run_until_complete(self.connect_loop())
        except Exception as e:
            self.get_logger().error(f"❌ Async loop error: {e}")
        finally:
            loop.close()

    async def connect_loop(self):
        ssl_ctx = ssl.create_default_context()
        ssl_ctx.check_hostname = False
        ssl_ctx.verify_mode = ssl.CERT_NONE

        while self.running and rclpy.ok():
            try:
                self.get_logger().info(f"🌍 Connecting to {self.target_uri} ...")

                # websockets 15.x 使用 additional_headers
                async with websockets.connect(
                    self.target_uri,
                    ssl=ssl_ctx,
                    additional_headers={"Authorization": self.auth_token},
                    ping_interval=None
                ) as ws:

                    self.get_logger().info("✅ Connected to Secure Cloud Bridge")
                    self.last_heartbeat = time.time()

                    recv_task = asyncio.create_task(self.receiver(ws))
                    ping_task = asyncio.create_task(self.heartbeat(ws))

                    await asyncio.gather(recv_task, ping_task, return_exceptions=True)

            except Exception as e:
                self.get_logger().warn(f"⚠️ Connection lost: {e}")
                await asyncio.sleep(self.reconnect_delay)

    async def receiver(self, ws):
        try:
            async for msg in ws:
                try:
                    data = json.loads(msg)
                except Exception:
                    continue

                if data.get("type") == "ping":
                    await ws.send(json.dumps({"type": "pong", "t": data.get("t", 0)}))
                    self.last_heartbeat = time.time()
                    continue

                if data.get("type") == "drive":
                    lin = float(data.get("linear", 0.0))
                    ang = float(data.get("angular", 0.0))
                    self.publish_cmd_vel(lin, ang)
                    latency = time.time() - data.get("t", time.time())
                    self.get_logger().info(f"📡 cmd_vel ← lin:{lin:.2f} ang:{ang:.2f} | RTT≈{latency*1000:.1f} ms")
        except websockets.ConnectionClosed:
            self.get_logger().warn("🔌 WebSocket connection closed")
        except Exception as e:
            self.get_logger().error(f"❌ Receiver error: {e}")

    async def heartbeat(self, ws):
        while self.running:
            if time.time() - self.last_heartbeat > 10:
                self.get_logger().warn("💔 Heartbeat timeout. Reconnecting...")
                await ws.close()
                break
            await asyncio.sleep(1.0)

    def publish_cmd_vel(self, linear, angular):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.publisher_.publish(msg)

    def destroy_node(self):
        self.running = False
        super().destroy_node()
        self.get_logger().info("🛑 Vehicle secure client stopped.")


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelReceiverSecure()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()