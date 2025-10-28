#!/usr/bin/env python3
import asyncio
import json
import ssl
import time
import websockets
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


# =====================================================
# Secure ROS2 Vehicle Client
#   - 接收云端控制桥 (control_bridge_secure.py) 的指令
#   - 发布到 /cmd_vel
#   - 自动断链重连 + 可视化日志输出
# =====================================================

class CmdVelReceiverSecure(Node):
    def __init__(self):
        super().__init__('cmdvel_receiver_secure')

        self.target_uri = "wss://139.180.169.115:9443"
        self.auth_token = "secure_token_12345"  # 必须与云端一致
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

        self.last_heartbeat = time.time()
        self.reconnect_delay = 3.0

        self.get_logger().info(f"🚗 Vehicle secure client initialized. Target: {self.target_uri}")

        # 启动异步循环线程（兼容 Python 3.10+）
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        loop.create_task(self.connect_loop())

        # 独立线程运行 asyncio 事件循环
        import threading
        self.thread = threading.Thread(target=loop.run_forever, daemon=True)
        self.thread.start()

    # =====================================================
    # 建立安全连接
    # =====================================================
    async def connect_loop(self):
        ssl_ctx = ssl.create_default_context()
        ssl_ctx.check_hostname = False
        ssl_ctx.verify_mode = ssl.CERT_NONE

        while rclpy.ok():
            try:
                self.get_logger().info(f"🌍 Connecting to {self.target_uri} ...")

                async with websockets.connect(
                    self.target_uri,
                    ssl=ssl_ctx,
                    extra_headers={"Authorization": self.auth_token},
                    ping_interval=None
                ) as ws:

                    self.get_logger().info("✅ Connected to Secure Cloud Bridge")
                    self.last_heartbeat = time.time()

                    # 开始两个协程：消息接收 + 心跳检测
                    recv_task = asyncio.create_task(self.receiver(ws))
                    ping_task = asyncio.create_task(self.heartbeat(ws))

                    await asyncio.gather(recv_task, ping_task)

            except Exception as e:
                self.get_logger().warn(f"⚠️ Connection lost: {e}")
                await asyncio.sleep(self.reconnect_delay)

    # =====================================================
    # 接收消息
    # =====================================================
    async def receiver(self, ws):
        async for msg in ws:
            try:
                data = json.loads(msg)
            except Exception:
                continue

            # === 心跳回应 ===
            if data.get("type") == "ping":
                await ws.send(json.dumps({"type": "pong", "t": data.get("t", 0)}))
                continue

            # === 驱动指令 ===
            if data.get("type") == "drive":
                lin = float(data.get("linear", 0.0))
                ang = float(data.get("angular", 0.0))
                self.publish_cmd_vel(lin, ang)
                latency = time.time() - data.get("t", time.time())
                self.get_logger().info(f"cmd_vel ← lin:{lin:.2f} ang:{ang:.2f} | RTT≈{latency*1000:.1f} ms")

    # =====================================================
    # 心跳任务：检测断开
    # =====================================================
    async def heartbeat(self, ws):
        while True:
            if time.time() - self.last_heartbeat > 10:
                self.get_logger().warn("💔 Heartbeat timeout. Reconnecting soon...")
                await ws.close()
                break
            await asyncio.sleep(1.0)

    # =====================================================
    # 发布 ROS2 /cmd_vel
    # =====================================================
    def publish_cmd_vel(self, linear, angular):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.publisher_.publish(msg)

    # =====================================================
    # 优雅关闭
    # =====================================================
    def destroy_node(self):
        super().destroy_node()
        self.get_logger().info("🛑 Vehicle secure client stopped.")


# =====================================================
# 主入口
# =====================================================
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
