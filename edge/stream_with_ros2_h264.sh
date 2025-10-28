#!/bin/bash

SERVER_IP="139.180.169.115"

# 摄像头配置数组
declare -a CAMERAS=(0 1 2 3)
declare -a STREAM_NAMES=("CameraFrontLeft" "CameraRear" "CameraFrontRight" "CameraFront")
declare -a ROS2_PORTS=(5000 5001 5002 5003)

echo "🚀 启动 ${#CAMERAS[@]} 路摄像头推流..."

for i in "${!CAMERAS[@]}"; do
    video_id="${CAMERAS[$i]}"
    stream_name="${STREAM_NAMES[$i]}"
    ros2_port="${ROS2_PORTS[$i]}"
    
    echo "📹 启动 Camera $((i+1)): /dev/video${video_id} -> ${stream_name} (ROS2 port ${ros2_port})"
    
    gst-launch-1.0 -v \
        v4l2src device=/dev/video${video_id} ! \
        'video/x-raw, format=UYVY, width=1920, height=1080, framerate=30/1' ! \
        nvvidconv ! \
        'video/x-raw(memory:NVMM), format=NV12' ! \
        tee name=t \
        \
        t. ! queue max-size-buffers=2 leaky=downstream ! \
        nvv4l2h264enc bitrate=3000000 preset-level=1 idrinterval=15 insert-sps-pps=true ! \
        h264parse ! \
        flvmux streamable=true ! \
        rtmpsink location="rtmp://${SERVER_IP}:1935/live/${stream_name} live=1" sync=false \
        \
        t. ! queue max-size-buffers=2 leaky=downstream ! \
        nvv4l2h264enc bitrate=2000000 preset-level=1 profile=0 insert-sps-pps=true ! \
        h264parse ! \
        rtph264pay config-interval=1 pt=$((96+i)) mtu=1400 ! \
        udpsink host=127.0.0.1 port=${ros2_port} sync=false async=false &
    
    # 保存进程 ID
    echo $! > /tmp/camera_${stream_name}.pid
    
    # 等待一下，避免同时启动太多进程
    sleep 2
done

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "✅ ${#CAMERAS[@]} 路摄像头推流已全部启动"
echo ""
echo "📺 云端监控:"
echo "   http://${SERVER_IP}:8080/webrtc.html"
echo ""
echo "🤖 ROS2 端口:"
for i in "${!CAMERAS[@]}"; do
    echo "   Camera $((i+1)): localhost:${ROS2_PORTS[$i]}"
done
echo ""
echo "🛑 停止推流:"
echo "   ./stop_all_streams.sh"
echo "   或: pkill -f gst-launch"
echo ""
echo "📊 查看进程:"
echo "   ps aux | grep gst-launch | grep -v grep"
echo ""
echo "📝 查看日志:"
for i in "${!CAMERAS[@]}"; do
    echo "   Camera $((i+1)) PID: $(cat /tmp/camera_${STREAM_NAMES[$i]}.pid 2>/dev/null || echo 'N/A')"
done
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

# 等待所有后台进程
wait