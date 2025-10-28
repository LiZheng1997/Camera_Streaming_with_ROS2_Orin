#!/bin/bash

echo "🚀 部署 SRS WebRTC 监控系统..."

# 停止旧容器
docker stop srs 2>/dev/null
docker rm srs 2>/dev/null

# 创建目录
mkdir -p ~/srs_monitor/{conf,html,logs}

# 创建 SRS 配置
cat > ~/srs_monitor/conf/srs.conf << 'SRSCONF'
listen              1935;
max_connections     1000;
daemon              off;
srs_log_tank        console;

# ---------------- HTTP API ----------------
http_api {
    enabled         on;
    listen          1985;
    crossdomain     on;
}

# ---------------- HTTP Server ----------------
http_server {
    enabled         on;
    listen          8080;
    dir             ./objs/nginx/html;
    crossdomain     on;
}

# ---------------- WebRTC UDP Server ----------------
rtc_server {
    enabled         on;
    listen          8000;                   # UDP 端口（WebRTC 媒体通道）
    candidate       139.180.169.115;  # 必须改成实际 IP，否则浏览器无法连接
}

# ---------------- RTMP & WebRTC ----------------
vhost __defaultVhost__ {
    # --- WebRTC 转换 ---
    rtc {
        enabled         on;
        rtmp_to_rtc     on;   # RTMP→WebRTC（浏览器拉流）
        rtc_to_rtmp     off;
    }

    # --- HTTP-FLV（保留，可选）---
    http_remux {
        enabled         on;
        mount           [vhost]/[app]/[stream].flv;
    }

    # --- HLS（保留，可选）---
    hls {
        enabled         on;
        hls_fragment    10;
        hls_window      60;
        hls_path        ./objs/nginx/html;
        hls_m3u8_file   [app]/[stream].m3u8;
        hls_ts_file     [app]/[stream]-[seq].ts;
    }
}

# ---------------- Stats ----------------
stats {
    network         0;
}

SRSCONF

# 启动 SRS
echo "📦 启动 SRS Docker 容器..."
docker run -d --restart=always --name srs \
  -p 1935:1935 \
  -p 1985:1985 \
  -p 8080:8080 \
  -p 8000:8000/udp \
  -v ~/srs_monitor/conf/srs.conf:/usr/local/srs/conf/srs.conf \
  -v ~/srs_monitor/html:/usr/local/srs/objs/nginx/html \
  -v ~/srs_monitor/logs:/usr/local/srs/logs \
  ossrs/srs:5 \
  ./objs/srs -c conf/srs.conf

# 等待启动
sleep 3

# 检查状态
if docker ps | grep -q srs; then
    echo ""
    echo "✅ 部署成功！"
    echo ""
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo "📺 WebRTC 监控: http://$(hostname -I | awk '{print $1}'):8080/webrtc.html"
    echo "🎛  SRS 控制台: http://$(hostname -I | awk '{print $1}'):8080/console/"
    echo "📡 RTMP 推流: rtmp://$(hostname -I | awk '{print $1}'):1935/live/camera1"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo ""
    echo "验证 WebRTC："
    echo "curl http://$(hostname -I | awk '{print $1}'):1985/api/v1/versions | grep rtc"
else
    echo "❌ 部署失败"
    docker logs srs
fi