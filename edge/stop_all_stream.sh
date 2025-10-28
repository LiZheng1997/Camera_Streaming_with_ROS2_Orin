#!/bin/bash

echo "🛑 停止所有摄像头推流..."

# 方法 1：通过 PID 文件停止
for pid_file in /tmp/camera_*.pid; do
    if [ -f "$pid_file" ]; then
        pid=$(cat "$pid_file")
        if kill -0 "$pid" 2>/dev/null; then
            echo "停止进程 $pid ($(basename $pid_file))"
            kill "$pid"
        fi
        rm "$pid_file"
    fi
done

# 方法 2：强制停止所有 gst-launch 进程
pkill -f "gst-launch.*rtmpsink"

# 等待进程结束
sleep 2

# 检查是否还有残留进程
remaining=$(ps aux | grep -E "gst-launch.*rtmpsink" | grep -v grep | wc -l)
if [ "$remaining" -gt 0 ]; then
    echo "⚠ 还有 $remaining 个进程残留，强制结束..."
    pkill -9 -f "gst-launch.*rtmpsink"
fi

echo "✅ 所有推流已停止"