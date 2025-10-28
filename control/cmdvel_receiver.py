#!/usr/bin/env python3
import asyncio, json, websockets, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CmdVelReceiver(Node):
    def __init__(self):
        super().__init__('cmdvel_receiver')

        # ========== 参数区 ==========
        self.server_uri = "ws://139.180.169.115:9090"  # ✅ 云端控制桥 WebSocket 地址
        self.heartbeat_timeout = 1.5   # 秒：若超过此时间未收到控制指令 -> 停车
        self.smooth_alpha = 0.2        # 平滑系数 (0.1-0.5)
        self.reconnect_interval = 3.0  # 断开后重连间隔
        # ============================

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.last_recv_time = 0.0
        self.desired = {"linear": 0.0, "angular": 0.0}
        self.current = {"linear": 0.0, "angular": 0.0}
        self.connected = False
        self.stop_published = False

        self.get_logger().info(f"🚗 CmdVelReceiver node started. Target server: {self.server_uri}")
        self.create_timer(0.05, self.publish_cmd_loop)  # 20Hz
        asyncio.create_task(self.connect_loop())

    # -------------------------------
    # 🔌 自动连接 + 接收数据
    # -------------------------------
    async def connect_loop(self):
        while rclpy.ok():
            try:
                self.get_logger().info(f"🌍 Connecting to {self.server_uri} ...")
                async with websockets.connect(self.server_uri, ping_interval=10, ping_timeout=5) as ws:
                    self.get_logger().info("✅ Connected to Cloud Control Server")
                    self.connected = True
                    self.stop_published = False
                    async for msg in ws:
                        data = json.loads(msg)
                        if data.get("type") == "drive":
                            self.last_recv_time = time.time()
                            self.desired["linear"] = float(data.get("linear", 0.0))
                            self.desired["angular"] = float(data.get("angular", 0.0))
            except Exception as e:
                self.connected = False
                self.get_logger().warn(f"⚠️ Connection lost: {e}")
                await asyncio.sleep(self.reconnect_interval)

    # -------------------------------
    # 🧠 平滑 & 超时安全停车
    # -------------------------------
    def publish_cmd_loop(self):
        now = time.time()
        twist = Twist()

        # 若超时未收到控制消息 -> 自动停车
        if self.connected and (now - self.last_recv_time > self.heartbeat_timeout):
            if not self.stop_published:
                self.get_logger().warn("⏱️ No heartbeat, stopping vehicle for safety.")
            self.desired["linear"] = 0.0
            self.desired["angular"] = 0.0
            self.stop_published = True

        # 平滑输出（指数滤波）
        for key in ["linear", "angular"]:
            self.current[key] = (
                (1 - self.smooth_alpha) * self.current[key]
                + self.smooth_alpha * self.desired[key]
            )

        twist.linear.x = self.current["linear"]
        twist.angular.z = self.current["angular"]
        self.pub.publish(twist)

        if abs(twist.linear.x) > 0.001 or abs(twist.angular.z) > 0.001:
            self.get_logger().info_throttle(
                1.0, f"cmd_vel → lin: {twist.linear.x:.2f}, ang: {twist.angular.z:.2f}"
            )

def main():
    rclpy.init()
    node = CmdVelReceiver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Interrupted by user, stopping...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    asyncio.run(main())
