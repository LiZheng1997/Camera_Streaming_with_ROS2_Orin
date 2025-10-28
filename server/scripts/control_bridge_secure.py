#!/usr/bin/env python3
import asyncio
import json
import ssl
import time
import os
import websockets
from websockets.server import serve


class ControlBridge:
    def __init__(self):
        self.active_clients = set()
        self.auth_tokens = {"secure_token_12345"}
        self.last_ping = {}
        self.heartbeat_timeout = 10.0  # 改为 10 秒，与客户端一致

        self.log_path = "/var/log/control_bridge_secure.log"
        os.makedirs(os.path.dirname(self.log_path), exist_ok=True)

        print("🚀 Secure Control Bridge initialized.")

    async def handler(self, websocket):
        peer = websocket.remote_address
        print(f"🔌 New connection from {peer}")

        # 身份验证
        try:
            headers = websocket.request_headers
            token = headers.get("Authorization", "")
            if token not in self.auth_tokens:
                print(f"🚫 Unauthorized client {peer}")
                await websocket.close(code=4001, reason="Unauthorized")
                return
        except Exception as e:
            print(f"⚠️ Auth error: {e}")
            await websocket.close()
            return

        # 注册客户端
        self.active_clients.add(websocket)
        self.last_ping[websocket] = time.time()
        print(f"✅ Client connected: {peer} (total: {len(self.active_clients)})")

        try:
            async for message in websocket:
                await self.process_message(websocket, message)
        except websockets.ConnectionClosed:
            print(f"🔌 Client {peer} connection closed")
        except Exception as e:
            print(f"⚠️ Handler error for {peer}: {e}")
        finally:
            self.active_clients.discard(websocket)
            self.last_ping.pop(websocket, None)
            print(f"❌ Client disconnected: {peer} (total: {len(self.active_clients)})")

    async def process_message(self, websocket, message):
        try:
            data = json.loads(message)
        except Exception:
            return

        # 驱动控制命令
        if data.get("type") == "drive":
            lin = data.get("linear", 0.0)
            ang = data.get("angular", 0.0)
            timestamp = data.get("t", time.time())
            
            msg = json.dumps({
                "type": "drive",
                "linear": lin,
                "angular": ang,
                "t": timestamp
            })
            await self.broadcast(msg, exclude=websocket)
            self.log(f"Drive cmd: lin={lin:.2f}, ang={ang:.2f}")

        # Pong 响应
        elif data.get("type") == "pong":
            self.last_ping[websocket] = time.time()

    async def broadcast(self, message, exclude=None):
        to_remove = []
        for ws in list(self.active_clients):
            if ws == exclude:
                continue
            try:
                await ws.send(message)
            except Exception as e:
                print(f"⚠️ Broadcast error to {ws.remote_address}: {e}")
                to_remove.append(ws)
        
        for ws in to_remove:
            self.active_clients.discard(ws)
            self.last_ping.pop(ws, None)

    async def ping_loop(self):
        while True:
            now = time.time()
            dead_clients = []

            for ws in list(self.active_clients):
                # 先检查超时
                last = self.last_ping.get(ws, 0)
                if now - last > self.heartbeat_timeout:
                    print(f"⏱️ Client {ws.remote_address} timeout (last pong: {now - last:.1f}s ago)")
                    dead_clients.append(ws)
                    continue

                # 发送 ping
                try:
                    await ws.send(json.dumps({"type": "ping", "t": now}))
                except Exception as e:
                    print(f"⚠️ Ping error to {ws.remote_address}: {e}")
                    dead_clients.append(ws)

            # 移除失效连接
            for ws in dead_clients:
                print(f"🗑️ Removing dead client: {ws.remote_address}")
                self.active_clients.discard(ws)
                self.last_ping.pop(ws, None)
                try:
                    await ws.close()
                except:
                    pass

            await asyncio.sleep(2.0)  # 每2秒检查一次

    def log(self, text):
        try:
            with open(self.log_path, "a") as f:
                f.write(f"[{time.strftime('%Y-%m-%d %H:%M:%S')}] {text}\n")
        except Exception as e:
            print(f"⚠️ Log error: {e}")


async def main():
    bridge = ControlBridge()

    # SSL 配置
    cert_dir = "/etc/ssl/srs"
    os.makedirs(cert_dir, exist_ok=True)
    cert_path = os.path.join(cert_dir, "server.crt")
    key_path = os.path.join(cert_dir, "server.key")

    if not os.path.exists(cert_path) or not os.path.exists(key_path):
        print("⚠️ SSL cert/key not found, generating self-signed pair...")
        os.system(
            f"openssl req -x509 -newkey rsa:2048 -keyout {key_path} "
            f"-out {cert_path} -days 365 -nodes -subj '/CN=139.180.169.115'"
        )
        print("✅ Self-signed certificate created")

    ssl_ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    ssl_ctx.load_cert_chain(cert_path, key_path)

    # 启动安全 WebSocket 服务器
    async with serve(
        bridge.handler,
        host="0.0.0.0",
        port=9443,
        ssl=ssl_ctx,
    ):
        print("🔒 Secure Control Bridge listening on wss://0.0.0.0:9443")
        print(f"📋 Log file: {bridge.log_path}")
        await bridge.ping_loop()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n🛑 Server stopped by user.")