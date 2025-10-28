#!/usr/bin/env python3
import asyncio
import websockets
import ssl
import json
import sys

async def test_wss():
    uri = "wss://139.180.169.115:9443"
    
    # 创建 SSL 上下文（跳过证书验证，因为是自签名证书）
    ssl_ctx = ssl.create_default_context()
    ssl_ctx.check_hostname = False
    ssl_ctx.verify_mode = ssl.CERT_NONE
    
    try:
        print(f"🔍 Testing connection to {uri}...")
        print(f"📦 websockets version: {websockets.__version__}")
        
        # websockets 15.x 使用 additional_headers
        async with websockets.connect(
            uri,
            ssl=ssl_ctx,
            additional_headers={"Authorization": "secure_token_12345"}
        ) as ws:
            print("✅ Connected successfully!")
            
            # 发送测试消息
            test_msg = json.dumps({"type": "test", "message": "hello"})
            await ws.send(test_msg)
            print(f"📤 Sent: {test_msg}")
            
            # 等待响应（最多等待5秒）
            try:
                response = await asyncio.wait_for(ws.recv(), timeout=5.0)
                print(f"📥 Received: {response}")
            except asyncio.TimeoutError:
                print("⏱️ No response received (timeout), but connection is OK")
            
            print("✅ WebSocket connection is working!")
            return True
            
    except ConnectionRefusedError:
        print("❌ Connection refused - server may not be running")
        return False
    except asyncio.TimeoutError:
        print("❌ Connection timeout - server may be unreachable")
        return False
    except websockets.exceptions.InvalidStatus as e:
        print(f"❌ Invalid status: {e}")
        print("   This might mean authentication failed or wrong endpoint")
        return False
    except OSError as e:
        print(f"❌ Network error: {e}")
        print("   Check if the server IP and port are correct")
        return False
    except Exception as e:
        print(f"❌ Connection failed: {type(e).__name__}: {e}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    try:
        result = asyncio.run(test_wss())
        sys.exit(0 if result else 1)
    except KeyboardInterrupt:
        print("\n⚠️ Test interrupted by user")
        sys.exit(1)