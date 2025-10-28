#!/bin/bash

echo "🛑 停止所有推流..."

# 停止所有 gst-launch 进程
pkill -f "gst-launch.*rtmpsink"

# 删除 PID 文件
rm -f /tmp/camera_*.pid

echo "✅ 已停止所有推流"