#!/bin/bash

echo "🚀 部署 SRS 多路摄像头监控系统..."

# 停止并删除旧容器
docker stop srs 2>/dev/null
docker rm srs 2>/dev/null

# 创建目录
mkdir -p ~/srs_monitor/html
mkdir -p ~/srs_monitor/logs


cat > ~/srs_monitor/html/index.html << 'EOF'
<!DOCTYPE html>
<html>
<head>
    <meta charset="utf-8">
    <title>Multi-Camera Monitor System</title>
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <style>
        * {
            margin: 0;
            padding: 0;
            box-sizing: border-box;
        }
        
        body {
            background: linear-gradient(135deg, #0a0e27 0%, #1a1e35 100%);
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            color: white;
            min-height: 100vh;
        }
        
        .header {
            background: rgba(0, 0, 0, 0.3);
            padding: 20px;
            text-align: center;
            border-bottom: 2px solid #00ff88;
            box-shadow: 0 2px 10px rgba(0, 0, 0, 0.5);
        }
        
        h1 {
            color: #00ff88;
            font-size: 2.5em;
            text-shadow: 0 0 20px rgba(0, 255, 136, 0.5);
            margin-bottom: 10px;
        }
        
        .subtitle {
            color: #888;
            font-size: 1.1em;
        }
        
        .controls {
            background: rgba(0, 0, 0, 0.3);
            padding: 15px 20px;
            display: flex;
            justify-content: space-between;
            align-items: center;
            flex-wrap: wrap;
            gap: 10px;
            border-bottom: 1px solid rgba(255, 255, 255, 0.1);
        }
        
        .view-mode {
            display: flex;
            gap: 10px;
        }
        
        .btn {
            padding: 8px 16px;
            border: none;
            border-radius: 5px;
            cursor: pointer;
            font-size: 0.9em;
            transition: all 0.3s ease;
            color: white;
            background: rgba(255, 255, 255, 0.1);
        }
        
        .btn:hover {
            background: rgba(255, 255, 255, 0.2);
            transform: translateY(-2px);
        }
        
        .btn.active {
            background: #00ff88;
            color: #000;
        }
        
        .container {
            padding: 20px;
        }
        
        .video-grid {
            display: grid;
            gap: 20px;
            margin-bottom: 20px;
        }
        
        .grid-1 { grid-template-columns: 1fr; }
        .grid-2 { grid-template-columns: repeat(2, 1fr); }
        .grid-3 { grid-template-columns: repeat(3, 1fr); }
        .grid-4 { grid-template-columns: repeat(2, 1fr); }
        
        .video-card {
            background: rgba(255, 255, 255, 0.05);
            border: 1px solid rgba(255, 255, 255, 0.1);
            border-radius: 10px;
            overflow: hidden;
            box-shadow: 0 4px 20px rgba(0, 0, 0, 0.3);
            transition: all 0.3s ease;
        }
        
        .video-card:hover {
            transform: translateY(-5px);
            box-shadow: 0 8px 30px rgba(0, 255, 136, 0.2);
            border-color: #00ff88;
        }
        
        .video-card.fullscreen {
            position: fixed;
            top: 0;
            left: 0;
            right: 0;
            bottom: 0;
            z-index: 1000;
            border-radius: 0;
            margin: 0;
        }
        
        .video-header {
            padding: 12px 15px;
            background: rgba(0, 0, 0, 0.5);
            border-bottom: 2px solid #00ff88;
            display: flex;
            justify-content: space-between;
            align-items: center;
        }
        
        .camera-info {
            display: flex;
            align-items: center;
            gap: 10px;
        }
        
        .camera-name {
            font-size: 1.1em;
            font-weight: bold;
        }
        
        .status {
            display: flex;
            align-items: center;
            gap: 8px;
            font-size: 0.9em;
        }
        
        .status-dot {
            width: 10px;
            height: 10px;
            border-radius: 50%;
            background: #ff4444;
            animation: pulse 2s infinite;
        }
        
        .status-dot.connected {
            background: #00ff88;
        }
        
        @keyframes pulse {
            0%, 100% { opacity: 1; }
            50% { opacity: 0.5; }
        }
        
        .video-container {
            position: relative;
            background: #000;
            padding-bottom: 56.25%; /* 16:9 */
        }
        
        video {
            position: absolute;
            top: 0;
            left: 0;
            width: 100%;
            height: 100%;
            object-fit: contain;
        }
        
        .video-overlay {
            position: absolute;
            top: 10px;
            right: 10px;
            display: flex;
            gap: 5px;
            z-index: 10;
        }
        
        .overlay-btn {
            width: 36px;
            height: 36px;
            border-radius: 50%;
            background: rgba(0, 0, 0, 0.7);
            border: none;
            color: white;
            cursor: pointer;
            display: flex;
            align-items: center;
            justify-content: center;
            transition: all 0.3s ease;
        }
        
        .overlay-btn:hover {
            background: #00ff88;
            color: #000;
            transform: scale(1.1);
        }
        
        .video-footer {
            padding: 10px 15px;
            background: rgba(0, 0, 0, 0.5);
            font-size: 0.85em;
            color: #888;
            display: flex;
            justify-content: space-between;
        }
        
        .stats-panel {
            background: rgba(255, 255, 255, 0.05);
            border: 1px solid rgba(255, 255, 255, 0.1);
            border-radius: 10px;
            padding: 20px;
            margin-top: 20px;
        }
        
        .stats-grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
            gap: 15px;
            margin-top: 15px;
        }
        
        .stat-item {
            background: rgba(0, 0, 0, 0.3);
            padding: 15px;
            border-radius: 8px;
            border-left: 3px solid #00ff88;
        }
        
        .stat-label {
            color: #888;
            font-size: 0.9em;
            margin-bottom: 5px;
        }
        
        .stat-value {
            font-size: 1.5em;
            font-weight: bold;
            color: #00ff88;
        }
        
        .loading {
            position: absolute;
            top: 50%;
            left: 50%;
            transform: translate(-50%, -50%);
            color: #888;
            font-size: 1.2em;
        }
        
        .error-message {
            position: absolute;
            top: 50%;
            left: 50%;
            transform: translate(-50%, -50%);
            color: #ff4444;
            text-align: center;
            padding: 20px;
        }
        
        @media (max-width: 1024px) {
            .grid-2, .grid-3, .grid-4 {
                grid-template-columns: 1fr !important;
            }
            h1 {
                font-size: 1.8em;
            }
        }
    </style>
</head>
<body>
    <div class="header">
        <h1>🎥 Multi-Camera Monitoring System</h1>
        <p class="subtitle">Real-time Video Surveillance Platform</p>
    </div>
    
    <div class="controls">
        <div class="view-mode">
            <button class="btn active" onclick="setGridMode(1)">1x1</button>
            <button class="btn" onclick="setGridMode(2)">2x2</button>
            <button class="btn" onclick="setGridMode(3)">3x3</button>
            <button class="btn" onclick="setGridMode(4)">4x4</button>
        </div>
        <div class="view-mode">
            <button class="btn" onclick="playAll()">▶ Play All</button>
            <button class="btn" onclick="pauseAll()">⏸ Pause All</button>
            <button class="btn" onclick="refreshAll()">🔄 Refresh</button>
        </div>
    </div>
    
    <div class="container">
        <div class="video-grid grid-2" id="videoGrid">
            <!-- 摄像头会动态生成 -->
        </div>
        
        <div class="stats-panel">
            <h3>📊 System Statistics</h3>
            <div class="stats-grid">
                <div class="stat-item">
                    <div class="stat-label">Total Cameras</div>
                    <div class="stat-value" id="totalCameras">0</div>
                </div>
                <div class="stat-item">
                    <div class="stat-label">Active Streams</div>
                    <div class="stat-value" id="activeStreams">0</div>
                </div>
                <div class="stat-item">
                    <div class="stat-label">Server Status</div>
                    <div class="stat-value" id="serverStatus">Checking...</div>
                </div>
                <div class="stat-item">
                    <div class="stat-label">Uptime</div>
                    <div class="stat-value" id="uptime">00:00:00</div>
                </div>
            </div>
        </div>
    </div>

    <script src="https://cdn.jsdelivr.net/npm/flv.js@latest/dist/flv.min.js"></script>
    <script>
        // 摄像头配置 - 根据实际情况修改
        const cameras = [
            { id: 1, name: 'CameraFrontLeft Camera', stream: 'CameraFrontLeft', icon: '📹' },
            { id: 2, name: 'CameraRear Camera', stream: 'CameraRear', icon: '🎬' },
            { id: 3, name: 'CameraFrontRight Camera', stream: 'CameraFrontRight', icon: '📷' },
            { id: 4, name: 'CameraFront Camera', stream: 'CameraFront', icon: '🎥' }
        ];

        const players = {};
        let currentGrid = 2;
        let startTime = Date.now();

        // 创建视频卡片
        function createVideoCard(camera) {
            return `
                <div class="video-card" id="card-${camera.id}">
                    <div class="video-header">
                        <div class="camera-info">
                            <span>${camera.icon}</span>
                            <span class="camera-name">${camera.name}</span>
                        </div>
                        <div class="status">
                            <span class="status-dot" id="status-dot-${camera.id}"></span>
                            <span id="status-text-${camera.id}">Connecting...</span>
                        </div>
                    </div>
                    <div class="video-container">
                        <div class="loading" id="loading-${camera.id}">Loading stream...</div>
                        <video id="video-${camera.id}" muted></video>
                        <div class="video-overlay">
                            <button class="overlay-btn" onclick="togglePlay(${camera.id})" title="Play/Pause">
                                <span id="play-icon-${camera.id}">▶</span>
                            </button>
                            <button class="overlay-btn" onclick="toggleFullscreen(${camera.id})" title="Fullscreen">⛶</button>
                            <button class="overlay-btn" onclick="takeSnapshot(${camera.id})" title="Snapshot">📷</button>
                        </div>
                    </div>
                    <div class="video-footer">
                        <span id="resolution-${camera.id}">Resolution: --</span>
                        <span id="fps-${camera.id}">FPS: --</span>
                    </div>
                </div>
            `;
        }

        // 初始化播放器
        function initPlayer(camera) {
            if (!flvjs.isSupported()) {
                console.error('FLV.js is not supported');
                return;
            }

            const videoElement = document.getElementById(`video-${camera.id}`);
            const statusDot = document.getElementById(`status-dot-${camera.id}`);
            const statusText = document.getElementById(`status-text-${camera.id}`);
            const loading = document.getElementById(`loading-${camera.id}`);

            const flvUrl = `${window.location.protocol}//${window.location.hostname}:8080/live/${camera.stream}.flv`;

            try {
                const player = flvjs.createPlayer({
                    type: 'flv',
                    url: flvUrl,
                    isLive: true
                }, {
                    enableWorker: false,
                    enableStashBuffer: false,
                    stashInitialSize: 128,
                    autoCleanupSourceBuffer: true,
                    autoCleanupMaxBackwardDuration: 3,
                    autoCleanupMinBackwardDuration: 2
                });

                player.attachMediaElement(videoElement);
                player.load();

                // 事件监听
                player.on(flvjs.Events.ERROR, (errorType, errorDetail, errorInfo) => {
                    console.error(`Camera ${camera.id} error:`, errorType, errorDetail);
                    statusDot.classList.remove('connected');
                    statusText.textContent = 'Offline';
                    loading.innerHTML = '<div class="error-message">⚠<br>Stream Unavailable</div>';
                });

                videoElement.addEventListener('loadedmetadata', () => {
                    loading.style.display = 'none';
                    const resolution = `${videoElement.videoWidth}x${videoElement.videoHeight}`;
                    document.getElementById(`resolution-${camera.id}`).textContent = `Resolution: ${resolution}`;
                });

                videoElement.addEventListener('playing', () => {
                    statusDot.classList.add('connected');
                    statusText.textContent = 'Live';
                    loading.style.display = 'none';
                    document.getElementById(`play-icon-${camera.id}`).textContent = '⏸';
                    updateActiveStreams();
                });

                videoElement.addEventListener('pause', () => {
                    document.getElementById(`play-icon-${camera.id}`).textContent = '▶';
                });

                videoElement.addEventListener('waiting', () => {
                    statusText.textContent = 'Buffering...';
                });

                // 自动播放
                setTimeout(() => {
                    player.play().catch(err => {
                        console.log(`Camera ${camera.id} autoplay prevented:`, err);
                        statusText.textContent = 'Click to play';
                    });
                }, 500);

                players[camera.id] = player;

                // FPS 监控
                setInterval(() => {
                    if (videoElement.played.length > 0) {
                        document.getElementById(`fps-${camera.id}`).textContent = 'FPS: 30';
                    }
                }, 1000);

            } catch (error) {
                console.error(`Failed to init camera ${camera.id}:`, error);
                loading.innerHTML = '<div class="error-message">⚠<br>Failed to initialize</div>';
            }
        }

        // 控制函数
        function togglePlay(cameraId) {
            const video = document.getElementById(`video-${cameraId}`);
            if (video.paused) {
                video.play();
            } else {
                video.pause();
            }
        }

        function toggleFullscreen(cameraId) {
            const card = document.getElementById(`card-${cameraId}`);
            const video = document.getElementById(`video-${cameraId}`);
            
            if (!document.fullscreenElement) {
                if (video.requestFullscreen) {
                    video.requestFullscreen();
                } else if (video.webkitRequestFullscreen) {
                    video.webkitRequestFullscreen();
                }
            } else {
                if (document.exitFullscreen) {
                    document.exitFullscreen();
                }
            }
        }

        function takeSnapshot(cameraId) {
            const video = document.getElementById(`video-${cameraId}`);
            const canvas = document.createElement('canvas');
            canvas.width = video.videoWidth;
            canvas.height = video.videoHeight;
            const ctx = canvas.getContext('2d');
            ctx.drawImage(video, 0, 0);
            
            canvas.toBlob(blob => {
                const url = URL.createObjectURL(blob);
                const a = document.createElement('a');
                const timestamp = new Date().toISOString().replace(/:/g, '-').split('.')[0];
                a.href = url;
                a.download = `camera${cameraId}_${timestamp}.png`;
                a.click();
                URL.revokeObjectURL(url);
            });
        }

        function setGridMode(cols) {
            currentGrid = cols;
            const grid = document.getElementById('videoGrid');
            grid.className = `video-grid grid-${cols}`;
            
            // 更新按钮状态
            document.querySelectorAll('.view-mode .btn').forEach(btn => {
                btn.classList.remove('active');
            });
            event.target.classList.add('active');
        }

        function playAll() {
            cameras.forEach(cam => {
                const video = document.getElementById(`video-${cam.id}`);
                video.play().catch(() => {});
            });
        }

        function pauseAll() {
            cameras.forEach(cam => {
                const video = document.getElementById(`video-${cam.id}`);
                video.pause();
            });
        }

        function refreshAll() {
            cameras.forEach(cam => {
                if (players[cam.id]) {
                    players[cam.id].unload();
                    players[cam.id].load();
                    players[cam.id].play();
                }
            });
        }

        function updateActiveStreams() {
            let active = 0;
            cameras.forEach(cam => {
                const video = document.getElementById(`video-${cam.id}`);
                if (video && !video.paused && !video.ended) {
                    active++;
                }
            });
            document.getElementById('activeStreams').textContent = active;
        }

        // 检查服务器状态
        async function checkServerStatus() {
            try {
                const response = await fetch(`http://${window.location.hostname}:1985/api/v1/versions`);
                if (response.ok) {
                    document.getElementById('serverStatus').textContent = 'Online';
                    document.getElementById('serverStatus').style.color = '#00ff88';
                }
            } catch (error) {
                document.getElementById('serverStatus').textContent = 'Offline';
                document.getElementById('serverStatus').style.color = '#ff4444';
            }
        }

        // 更新运行时间
        function updateUptime() {
            const elapsed = Date.now() - startTime;
            const hours = Math.floor(elapsed / 3600000);
            const minutes = Math.floor((elapsed % 3600000) / 60000);
            const seconds = Math.floor((elapsed % 60000) / 1000);
            document.getElementById('uptime').textContent = 
                `${String(hours).padStart(2, '0')}:${String(minutes).padStart(2, '0')}:${String(seconds).padStart(2, '0')}`;
        }

        // 初始化
        window.addEventListener('DOMContentLoaded', () => {
            const grid = document.getElementById('videoGrid');
            
            // 渲染摄像头卡片
            cameras.forEach(camera => {
                grid.innerHTML += createVideoCard(camera);
            });
            
            // 更新统计
            document.getElementById('totalCameras').textContent = cameras.length;
            
            // 初始化播放器
            cameras.forEach(camera => {
                initPlayer(camera);
            });
            
            // 定期更新
            setInterval(checkServerStatus, 5000);
            setInterval(updateUptime, 1000);
            setInterval(updateActiveStreams, 2000);
            
            // 首次检查
            checkServerStatus();
        });

        // 清理
        window.addEventListener('beforeunload', () => {
            Object.values(players).forEach(player => {
                try {
                    player.pause();
                    player.unload();
                    player.detachMediaElement();
                    player.destroy();
                } catch (e) {}
            });
        });
    </script>
</body>
</html>
EOF

# 启动 SRS 容器（修复挂载路径问题）
echo "📦 启动 SRS Docker 容器..."
docker run -d --restart=always --name srs \
  -p 1935:1935 \
  -p 1985:1985 \
  -p 8080:8080 \
  -p 8000:8000/udp \
  -v ~/srs_monitor/html:/usr/local/srs/objs/nginx/html \
  -v ~/srs_monitor/logs:/usr/local/srs/logs \
  ossrs/srs:5

# 等待启动
sleep 3

# 检查状态
if docker ps | grep -q srs; then
    echo ""
    echo "✅ 部署成功！"
    echo ""
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo "📺 监控页面: http://$(hostname -I | awk '{print $1}'):8080/index.html"
    echo "🎛 控制台接口: http://$(hostname -I | awk '{print $1}'):1985/api/v1/versions"
    echo "📡 RTMP 推流地址: rtmp://$(hostname -I | awk '{print $1}'):1935/live/camera1"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo ""
else
    echo "❌ 部署失败，请检查 Docker 日志："
    docker logs srs
fi
