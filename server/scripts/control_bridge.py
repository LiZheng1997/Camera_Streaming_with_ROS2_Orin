#!/usr/bin/env python3
import asyncio, json, websockets, rclpy
from geometry_msgs.msg import Twist

rclpy.init()
node = rclpy.create_node('web_control_bridge')
pub = node.create_publisher(Twist, '/cmd_vel', 10)

async def handler(websocket):
    node.get_logger().info("✅ WebSocket client connected")
    async for msg in websocket:
        try:
            data = json.loads(msg)
        except Exception as e:
            node.get_logger().error(f"Invalid JSON: {e}")
            continue

        twist = Twist()
        if data.get("type") == "drive":
            twist.linear.x = float(data.get("linear", 0.0))
            twist.angular.z = float(data.get("angular", 0.0))
        else:
            cmd = data.get("command")
            if cmd == "forward": twist.linear.x = 0.5
            elif cmd == "backward": twist.linear.x = -0.5
            elif cmd == "left": twist.angular.z = 1.0
            elif cmd == "right": twist.angular.z = -1.0
            elif cmd == "stop": pass
        pub.publish(twist)

async def main():
    # 🚫 不再传 extra_headers，兼容所有 asyncio 事件循环
    async with websockets.serve(handler, "0.0.0.0", 9090):
        node.get_logger().info("🌐 WebSocket Control Bridge running on :9090")
        await asyncio.Future()  # 保持运行

if __name__ == "__main__":
    asyncio.run(main())
