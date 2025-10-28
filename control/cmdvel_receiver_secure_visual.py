#!/usr/bin/env python3
import asyncio, json, websockets, time, ssl, csv, os, threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

# ---------- GPIO 工具 ----------
class GPIO:
    def __init__(self, pin, active_high=True):
        self.pin = pin
        self.path = f"/sys/class/gpio/gpio{pin}"
        self.active_high = active_high
        self.available = os.path.exists("/sys/class/gpio")

        if not self.available:
            print(f"⚠️ GPIO system not available, pin {pin} inactive.")
            return

        if not os.path.exists(self.path):
            try:
                with open("/sys/class/gpio/export", "w") as f:
                    f.write(str(pin))
            except Exception:
                pass

        try:
            with open(f"{self.path}/direction", "w") as f:
                f.write("out")
        except Exception:
            pass

    def set(self, state: bool):
        if not self.available:
            return
        try:
            with open(f"{self.path}/value", "w") as f:
                val = "1" if (state == self.active_high) else "0"
                f.write(val)
        except Exception:
            pass


class CmdVelReceiverSecure(Node):
    def __init__(self):
        super().__init__('cmdvel_receiver_secure_visual')

        # ======== 参数配置 ========
        self.server_uri = "wss://139.180.169.115:9443"
        self.auth_token = "secure_token_12345"
        self.heartbeat_timeout = 1.5
        self.smooth_alpha = 0.2
        self.reconnect_interval = 3.0
        self.log_dir = "/var/log/ros"

        # ======== ROS 接口 ========
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.safety_pub = self.create_publisher(Bool, "/vehicle/safety_stop", 10)

        # ======== GPIO 控制 ========
        # 红灯 → 断线；绿灯 → 在线；蜂鸣器 → 报警
        self.led_green = GPIO(17)   # GPIO17 (Pin 11)
        self.led_red = GPIO(27)     # GPIO27 (Pin 13)
        self.buzzer = GPIO(22)      # GPIO22 (Pin 15)

        # ======== 状态变量 ========
        self.desired = {"linear": 0.0, "angular": 0.0}
        self.current = {"linear": 0.0, "angular": 0.0}
        self.last_recv = 0.0
        self.connected = False
        self.stop_published = False
        self.last_rtt_ms = 0.0
        self.rtt_samples = []

        os.makedirs(self.log_dir, exist_ok=True)
        self.log_path = os.path.join(self.log_dir, "cmdvel_secure.csv")
        if not os.path.exists(self.log_path):
            with open(self.log_path, "w", newline="") as f:
                csv.writer(f).writerow(["timestamp", "linear", "angular", "rtt_ms", "connected"])

        self.create_timer(0.05, self.publish_loop)

        # ✅ 后台异步 WebSocket 线程
        self.thread = threading.Thread(target=self._run_async_loop, daemon=True)
        self.thread.start()

        self.get_logger().info(f"🚗 Vehicle secure client initialized. Target: {self.server_uri}")

    # ===================================================
    def _run_async_loop(self):
        asyncio.run(self.connect_loop())

    async def connect_loop(self):
        ssl_ctx = ssl._create_unverified_context()
        while rclpy.ok():
            try:
                self._set_leds(online=False)
                self.get_logger().info(f"🌍 Connecting to {self.server_uri} ...")
                async with websockets.connect(
                    self.server_uri,
                    ssl=ssl_ctx,
                    ping_interval=None,
                    extra_headers={"Authorization": self.auth_token},
                ) as ws:
                    self.get_logger().info("✅ Connected to Secure Cloud Bridge")
                    self.connected = True
                    self.stop_published = False
                    self._set_leds(online=True)
                    await self.recv_loop(ws)
            except Exception as e:
                self.connected = False
                self.get_logger().warn(f"⚠️ Connection lost: {e}")
                self._publish_stop()
                self._set_leds(online=False)
                self._beep()
                await asyncio.sleep(self.reconnect_interval)

    async def recv_loop(self, ws):
        async for msg in ws:
            data = json.loads(msg)
            if data.get("type") == "drive":
                self.last_recv = time.time()
                self.desired["linear"] = float(data.get("linear", 0.0))
                self.desired["angular"] = float(data.get("angular", 0.0))
                self._log_cmd(self.desired["linear"], self.desired["angular"])
            elif data.get("type") == "ping":
                t0 = data.get("t", 0)
                rtt = time.time() - t0
                self.last_rtt_ms = round(rtt * 1000, 1)
                self.rtt_samples.append(self.last_rtt_ms)
                if len(self.rtt_samples) > 30:
                    self.rtt_samples.pop(0)
                await ws.send(json.dumps({"type": "pong"}))

    # ===================================================
    def publish_loop(self):
        now = time.time()
        twist = Twist()

        if self.connected and (now - self.last_recv > self.heartbeat_timeout):
            if not self.stop_published:
                self.get_logger().warn("⏱️ No heartbeat, auto stopping.")
            self.desired = {"linear": 0.0, "angular": 0.0}
            self.stop_published = True
            self.safety_pub.publish(Bool(data=True))

        if not self.connected:
            self._publish_stop()
            return

        for key in ["linear", "angular"]:
            self.current[key] = (
                (1 - self.smooth_alpha) * self.current[key]
                + self.smooth_alpha * self.desired[key]
            )

        twist.linear.x = self.current["linear"]
        twist.angular.z = self.current["angular"]
        self.cmd_pub.publish(twist)

        avg_rtt = sum(self.rtt_samples) / len(self.rtt_samples) if self.rtt_samples else 0
        self.get_logger().info_throttle(
            2.0, f"cmd_vel → lin:{twist.linear.x:.2f} ang:{twist.angular.z:.2f} | RTT≈{avg_rtt:.1f}ms"
        )

    # ===================================================
    def _publish_stop(self):
        twist = Twist()
        twist.linear.x = twist.angular.z = 0.0
        self.cmd_pub.publish(twist)
        self.safety_pub.publish(Bool(data=True))

    def _log_cmd(self, lin, ang):
        with open(self.log_path, "a", newline="") as f:
            csv.writer(f).writerow([time.time(), lin, ang, self.last_rtt_ms, self.connected])

    # ---------- GPIO 控制 ----------
    def _set_leds(self, online: bool):
        """根据连接状态切换LED指示灯"""
        if online:
            self.led_green.set(True)
            self.led_red.set(False)
        else:
            self.led_green.set(False)
            self.led_red.set(True)

    def _beep(self, duration=1.0):
        """断线时鸣叫一声"""
        self.buzzer.set(True)
        threading.Timer(duration, lambda: self.buzzer.set(False)).start()


def main():
    rclpy.init()
    node = CmdVelReceiverSecure()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
