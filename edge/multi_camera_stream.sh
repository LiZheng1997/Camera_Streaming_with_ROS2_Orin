#!/bin/bash

# 配置
SERVER_IP="139.180.169.115"  # 修改为你的服务器 IP

# 摄像头配置
declare -A CAMERAS=(
    [0]="camera1"  # /dev/video0 -> camera1
    [1]="camera2"  # /dev/video1 -> camera2
    [2]="camera3"  # /dev/video2 -> camera3
    [3]="camera4"  # /dev/video3 -> camera4
)

echo "🚀 启动多路摄像头推流..."

# 为每个摄像头启动推流进程
for video_id in "${!CAMERAS[@]}"; do
    stream_name="${CAMERAS[$video_id]}"
    
    echo "📹 启动 /dev/video${video_id} -> ${stream_name}"
    
    gst-launch-1.0 -v \
        v4l2src device=/dev/video${video_id} ! \
        'video/x-raw, format=UYVY, width=1920, height=1080, framerate=30/1' ! \
        nvvidconv ! \
        'video/x-raw(memory:NVMM), format=NV12' ! \
        nvv4l2h264enc bitrate=3000000 preset-level=1 insert-sps-pps=true idrinterval=30 ! \
        h264parse ! \
        flvmux streamable=true ! \
        rtmpsink location="rtmp://${SERVER_IP}:1935/live/${stream_name} live=1" sync=false &
    
    # 保存进程 ID
    echo $! > /tmp/camera_${video_id}.pid
    
    sleep 2
done

echo "✅ 所有摄像头已启动"
echo "使用 'pkill -f gst-launch' 停止所有推流"