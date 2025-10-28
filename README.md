# Camera_Streaming_with_ROS2_Orin

This Repo is designed for camera streaming to a public cloud server with ROS2 support on Jetson Orin.

# Feature:

* [X] Multi-Cameras Streaming
* [X] Remote Driving UI
* [X] Local streaming for publishing ROS2 image topics
* [X] VIC/NVENC/NVDEC hardware acceleration

# Workflow

Jetson Orin (RTMP streaming) → SRS服务器 (RTMP converts to WebRTC) → 浏览器 (WebRTC video play)

# Deploy Steps

## Server

Using a docker image provided by us to run a demo. you need to pull the docker image first.

$ docker pull xxxx

### 1. Deploy SRS on a public cloud server

Currently, I am using a Vultr Cloud Compute for a demostration.

Overview of configurations:

vCPU/s: 1 vCPU

RAM: 1024.00 MB

Storage: 25 GB NVMe

Bandwidth: 2048 GB/Month

Use the following command to deploy a SRS service.

`$ bash deploy_webrtc.sh`

## Jetson Orin

### 1. Start camera streaming on Jetson Orin

### Commands

```
# 查看运行中的进程
ps aux | grep gst-launch | grep -v grep


# 检查服务器是否收到流
curl http://YOUR_SERVER_IP:1985/api/v1/streams/ | jq
```

## G29 Controller
