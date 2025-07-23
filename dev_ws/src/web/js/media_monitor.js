/**
 * ROS2 MQTT 多媒体监控系统
 * 支持图像、视频流、深度图、点云等多媒体数据的实时监控和显示
 */

class MediaMonitor {
    constructor() {
        this.mqttClient = null;
        this.isConnected = false;
        this.subscribedTopics = new Map();
        this.mediaStreams = new Map();
        this.messageCount = 0;
        this.dataTransferSize = 0;
        this.lastUpdateTime = Date.now();
        this.refreshRate = 10;
        this.isRecording = false;
        this.recordedFrames = [];
        
        this.initializeElements();
        this.setupEventListeners();
        this.updateStats();
        
        // 定期更新数据传输率
        setInterval(() => this.updateDataRate(), 1000);
    }

    initializeElements() {
        // 连接控制
        this.brokerHostInput = document.getElementById('brokerHost');
        this.brokerPortInput = document.getElementById('brokerPort');
        this.baseTopicInput = document.getElementById('baseTopic');
        this.connectBtn = document.getElementById('connectBtn');
        this.disconnectBtn = document.getElementById('disconnectBtn');
        this.connectionStatus = document.getElementById('connectionStatus');
        
        // 订阅管理
        this.newTopicInput = document.getElementById('newTopicInput');
        this.topicTypeSelect = document.getElementById('topicType');
        this.addTopicBtn = document.getElementById('addTopicBtn');
        this.subscribedTopicsDiv = document.getElementById('subscribedTopics');
        
        // 显示区域
        this.mediaGrid = document.getElementById('mediaGrid');
        
        // 控制面板
        this.refreshRateSlider = document.getElementById('refreshRate');
        this.refreshRateValue = document.getElementById('refreshRateValue');
        this.imageQualitySelect = document.getElementById('imageQuality');
        this.autoResizeCheck = document.getElementById('autoResize');
        this.showTimestampCheck = document.getElementById('showTimestamp');
        this.showMetadataCheck = document.getElementById('showMetadata');
        this.fullscreenBtn = document.getElementById('fullscreenBtn');
        this.saveImageBtn = document.getElementById('saveImageBtn');
        this.recordBtn = document.getElementById('recordBtn');
        
        // 统计信息
        this.subscribedCountSpan = document.getElementById('subscribedCount');
        this.messageCountSpan = document.getElementById('messageCount');
        this.activeStreamsSpan = document.getElementById('activeStreams');
        this.dataRateSpan = document.getElementById('dataRate');
        
        // 全屏查看器
        this.fullscreenViewer = document.getElementById('fullscreenViewer');
        this.fullscreenImage = document.getElementById('fullscreenImage');
        this.fullscreenVideo = document.getElementById('fullscreenVideo');
        this.fullscreenCanvas = document.getElementById('fullscreenCanvas');
        this.exitFullscreenBtn = document.getElementById('exitFullscreenBtn');
        
        // 配置折叠
        this.toggleConfigBtn = document.getElementById('toggleConfig');
        this.configContent = document.getElementById('configContent');
    }

    setupEventListeners() {
        // 连接控制
        this.connectBtn.addEventListener('click', () => this.connectToMQTT());
        this.disconnectBtn.addEventListener('click', () => this.disconnectFromMQTT());
        
        // 订阅管理
        this.addTopicBtn.addEventListener('click', () => this.addTopicSubscription());
        this.newTopicInput.addEventListener('keypress', (e) => {
            if (e.key === 'Enter') this.addTopicSubscription();
        });
        
        // 控制面板
        this.refreshRateSlider.addEventListener('input', (e) => {
            this.refreshRate = parseInt(e.target.value);
            this.refreshRateValue.textContent = this.refreshRate;
        });
        
        this.fullscreenBtn.addEventListener('click', () => this.toggleFullscreen());
        this.saveImageBtn.addEventListener('click', () => this.saveCurrentImages());
        this.recordBtn.addEventListener('click', () => this.toggleRecording());
        this.exitFullscreenBtn.addEventListener('click', () => this.exitFullscreen());
        
        // 配置折叠
        this.toggleConfigBtn.addEventListener('click', () => this.toggleConfig());
        
        // 预设话题
        document.querySelectorAll('.preset-btn').forEach(btn => {
            btn.addEventListener('click', (e) => {
                const topic = e.target.dataset.topic;
                const type = e.target.dataset.type;
                this.newTopicInput.value = topic;
                this.topicTypeSelect.value = type;
                this.addTopicSubscription();
            });
        });
        
        // 键盘快捷键
        document.addEventListener('keydown', (e) => {
            if (e.key === 'Escape') this.exitFullscreen();
            if (e.ctrlKey && e.key === 's') {
                e.preventDefault();
                this.saveCurrentImages();
            }
        });
    }

    async connectToMQTT() {
        const host = this.brokerHostInput.value || 'localhost';
        const port = parseInt(this.brokerPortInput.value) || 9001;
        
        try {
            this.updateConnectionStatus('connecting');
            this.showNotification('正在连接MQTT服务器...', 'info');
            
            // 创建MQTT客户端连接
            const clientId = `media_monitor_${Math.random().toString(16).substr(2, 8)}`;
            const brokerUrl = `ws://${host}:${port}/mqtt`;
            
            this.mqttClient = mqtt.connect(brokerUrl, {
                clientId: clientId,
                clean: true,
                connectTimeout: 5000,
                reconnectPeriod: 5000
            });
            
            this.mqttClient.on('connect', () => {
                this.isConnected = true;
                this.updateConnectionStatus('connected');
                this.connectBtn.disabled = true;
                this.disconnectBtn.disabled = false;
                this.showNotification('MQTT连接成功', 'success');
            });
            
            this.mqttClient.on('error', (error) => {
                console.error('MQTT连接错误:', error);
                this.showNotification(`MQTT连接错误: ${error.message}`, 'error');
                this.updateConnectionStatus('disconnected');
            });
            
            this.mqttClient.on('message', (topic, message) => {
                this.handleMQTTMessage(topic, message);
            });
            
            this.mqttClient.on('disconnect', () => {
                this.isConnected = false;
                this.updateConnectionStatus('disconnected');
                this.connectBtn.disabled = false;
                this.disconnectBtn.disabled = true;
                this.showNotification('MQTT连接已断开', 'warning');
            });
            
        } catch (error) {
            console.error('连接失败:', error);
            this.showNotification(`连接失败: ${error.message}`, 'error');
            this.updateConnectionStatus('disconnected');
        }
    }

    disconnectFromMQTT() {
        if (this.mqttClient) {
            this.mqttClient.end();
            this.mqttClient = null;
        }
        this.isConnected = false;
        this.updateConnectionStatus('disconnected');
        this.connectBtn.disabled = false;
        this.disconnectBtn.disabled = true;
        this.showNotification('已断开MQTT连接', 'info');
    }

    updateConnectionStatus(status) {
        const statusDot = this.connectionStatus.querySelector('.status-dot');
        const statusText = this.connectionStatus.querySelector('.status-text');
        
        statusDot.className = `status-dot ${status}`;
        
        switch (status) {
            case 'connected':
                statusText.textContent = '已连接';
                break;
            case 'connecting':
                statusText.textContent = '连接中...';
                break;
            case 'disconnected':
                statusText.textContent = '未连接';
                break;
        }
    }

    addTopicSubscription() {
        const topic = this.newTopicInput.value.trim();
        const type = this.topicTypeSelect.value;
        
        if (!topic) {
            this.showNotification('请输入有效的MQTT主题', 'warning');
            return;
        }
        
        if (!this.isConnected) {
            this.showNotification('请先连接MQTT服务器', 'warning');
            return;
        }
        
        if (this.subscribedTopics.has(topic)) {
            this.showNotification('该主题已订阅', 'warning');
            return;
        }
        
        try {
            this.mqttClient.subscribe(topic, (error) => {
                if (error) {
                    this.showNotification(`订阅失败: ${error.message}`, 'error');
                } else {
                    this.subscribedTopics.set(topic, { type, lastMessage: null });
                    this.addTopicToUI(topic, type);
                    this.createMediaCard(topic, type);
                    this.showNotification(`已订阅主题: ${topic}`, 'success');
                    this.updateStats();
                    
                    // 清空输入
                    this.newTopicInput.value = '';
                }
            });
        } catch (error) {
            this.showNotification(`订阅错误: ${error.message}`, 'error');
        }
    }

    removeTopicSubscription(topic) {
        if (this.mqttClient && this.subscribedTopics.has(topic)) {
            this.mqttClient.unsubscribe(topic);
            this.subscribedTopics.delete(topic);
            this.mediaStreams.delete(topic);
            
            // 从UI移除
            document.querySelector(`[data-topic="${topic}"]`)?.remove();
            document.querySelector(`[data-media-topic="${topic}"]`)?.remove();
            
            this.showNotification(`已取消订阅: ${topic}`, 'info');
            this.updateStats();
        }
    }

    addTopicToUI(topic, type) {
        const topicItem = document.createElement('div');
        topicItem.className = 'topic-item';
        topicItem.dataset.topic = topic;
        
        topicItem.innerHTML = `
            <div class="topic-info">
                <div class="topic-name">${topic}</div>
                <span class="topic-type">${this.getTypeDisplayName(type)}</span>
            </div>
            <button class="remove-topic-btn" onclick="mediaMonitor.removeTopicSubscription('${topic}')">
                ✕
            </button>
        `;
        
        this.subscribedTopicsDiv.appendChild(topicItem);
    }

    createMediaCard(topic, type) {
        const mediaCard = document.createElement('div');
        mediaCard.className = 'media-card';
        mediaCard.dataset.mediaTopic = topic;
        
        const typeIcon = this.getTypeIcon(type);
        
        mediaCard.innerHTML = `
            <div class="media-header">
                <div class="media-title">
                    ${typeIcon} ${topic}
                </div>
                <div class="media-controls">
                    <button class="media-control-btn" onclick="mediaMonitor.toggleMediaFullscreen('${topic}')" title="全屏">
                        ⛶
                    </button>
                    <button class="media-control-btn" onclick="mediaMonitor.saveMediaContent('${topic}')" title="保存">
                        💾
                    </button>
                    <button class="media-control-btn" onclick="mediaMonitor.refreshMedia('${topic}')" title="刷新">
                        🔄
                    </button>
                </div>
            </div>
            <div class="media-container" id="media-${this.sanitizeTopicId(topic)}">
                <div class="media-placeholder">
                    <div class="media-placeholder-icon">${typeIcon}</div>
                    <div>等待${this.getTypeDisplayName(type)}数据...</div>
                </div>
            </div>
            <div class="media-info">
                <div class="timestamp">
                    ⏱️ <span id="timestamp-${this.sanitizeTopicId(topic)}">--:--:--</span>
                </div>
                <div class="metadata" id="metadata-${this.sanitizeTopicId(topic)}">
                    <!-- 元数据将在这里显示 -->
                </div>
            </div>
        `;
        
        this.mediaGrid.appendChild(mediaCard);
    }

    handleMQTTMessage(topic, message) {
        this.messageCount++;
        this.dataTransferSize += message.length;
        
        if (!this.subscribedTopics.has(topic)) return;
        
        const topicInfo = this.subscribedTopics.get(topic);
        topicInfo.lastMessage = Date.now();
        
        try {
            // 解析JSON消息
            let messageData;
            try {
                const messageText = message.toString();
                messageData = JSON.parse(messageText);
                console.log(`收到${topic}的JSON消息:`, messageData);
            } catch (parseError) {
                console.error('JSON解析失败:', parseError);
                // 尝试作为直接的Base64字符串处理
                const messageText = message.toString();
                if (messageText.length > 100) {
                    messageData = messageText;
                } else {
                    console.error('无法处理的消息格式');
                    return;
                }
            }
            
            // 处理不同的消息格式
            let processedData = null;
            
            if (typeof messageData === 'string') {
                // 直接的Base64字符串
                processedData = {
                    type: 'image',
                    image_data: messageData,
                    timestamp: Date.now(),
                    source: 'direct_base64'
                };
            } else if (messageData && typeof messageData === 'object') {
                // JSON对象格式
                if (messageData.data && typeof messageData.data === 'object') {
                    // 检查data字段中的结构
                    const dataField = messageData.data;
                    
                    if (dataField.encoding === 'base64' && dataField.data) {
                        // 新格式：包含encoding字段
                        processedData = {
                            type: this.inferDataType(topic, messageData),
                            image_data: dataField.data,
                            width: dataField.width,
                            height: dataField.height,
                            encoding: dataField.data_type || 'unknown',
                            original_size: dataField.original_size,
                            timestamp: Date.now(),
                            source: 'structured_base64'
                        };
                    } else if (dataField.type === 'uint8_array' && dataField.data) {
                        // MQTT接口的自定义序列化格式
                        processedData = {
                            type: this.inferDataType(topic, messageData),
                            image_data: dataField.data,
                            size: dataField.size,
                            encoding: 'uint8_array',
                            timestamp: Date.now(),
                            source: 'mqtt_uint8_array'
                        };
                    }
                } else if (messageData.image_data) {
                    // 直接包含image_data字段
                    processedData = {
                        type: messageData.type || this.inferDataType(topic, messageData),
                        image_data: messageData.image_data,
                        width: messageData.width,
                        height: messageData.height,
                        encoding: messageData.encoding,
                        timestamp: messageData.timestamp || Date.now(),
                        source: 'direct_image_data'
                    };
                } else if (messageData.data && typeof messageData.data === 'string') {
                    // data字段是字符串（可能是Base64）
                    processedData = {
                        type: this.inferDataType(topic, messageData),
                        image_data: messageData.data,
                        timestamp: Date.now(),
                        source: 'data_string'
                    };
                }
                
                // 处理其他媒体类型
                if (!processedData) {
                    if (messageData.points && Array.isArray(messageData.points)) {
                        // 点云数据
                        processedData = {
                            type: 'pointcloud',
                            points: messageData.points,
                            point_count: messageData.point_count,
                            width: messageData.width,
                            height: messageData.height,
                            frame_id: messageData.frame_id,
                            timestamp: messageData.timestamp || Date.now(),
                            source: 'pointcloud_data'
                        };
                    } else if (messageData.depth_data) {
                        // 深度图数据
                        processedData = {
                            type: 'depth',
                            depth_data: messageData.depth_data,
                            width: messageData.width,
                            height: messageData.height,
                            min_depth: messageData.min_depth,
                            max_depth: messageData.max_depth,
                            timestamp: messageData.timestamp || Date.now(),
                            source: 'depth_data'
                        };
                    }
                }
            }
            
            if (processedData) {
                console.log(`处理后的数据:`, processedData);
                this.displayMediaContent(topic, processedData, topicInfo.type);
                this.updateMediaTimestamp(topic);
                
                // 如果正在录制，保存帧
                if (this.isRecording) {
                    this.recordFrame(topic, processedData);
                }
            } else {
                console.warn('无法处理的消息格式:', messageData);
                this.showNotification(`无法处理话题 ${topic} 的数据格式`, 'warning');
            }
            
        } catch (error) {
            console.error('处理MQTT消息错误:', error);
            this.showNotification(`消息处理错误: ${error.message}`, 'error');
        }
        
        this.updateStats();
    }

    inferDataType(topic, messageData) {
        // 根据话题名称和消息内容推断数据类型
        const topicLower = topic.toLowerCase();
        
        if (topicLower.includes('depth')) {
            return 'depth';
        } else if (topicLower.includes('pointcloud') || topicLower.includes('lidar')) {
            return 'pointcloud';
        } else if (topicLower.includes('video') || topicLower.includes('stream')) {
            return 'video';
        } else if (topicLower.includes('image') || topicLower.includes('camera')) {
            return 'image';
        }
        
        // 根据消息内容推断
        if (messageData && messageData.points) {
            return 'pointcloud';
        } else if (messageData && messageData.depth_data) {
            return 'depth';
        } else if (messageData && (messageData.image_data || messageData.data)) {
            return 'image';
        }
        
        return 'image'; // 默认为图像
    }

    displayMediaContent(topic, data, type) {
        const containerId = `media-${this.sanitizeTopicId(topic)}`;
        const container = document.getElementById(containerId);
        
        if (!container) return;
        
        // 清除占位符
        container.innerHTML = '';
        
        switch (type) {
            case 'image':
                this.displayImage(container, data, topic);
                break;
            case 'video':
                this.displayVideo(container, data, topic);
                break;
            case 'depth':
                this.displayDepthImage(container, data, topic);
                break;
            case 'pointcloud':
                this.displayPointCloud(container, data, topic);
                break;
            default:
                this.displayRawData(container, data, topic);
        }
        
        // 更新元数据
        this.updateMetadata(topic, data);
    }

    displayImage(container, data, topic) {
        let imageElement = container.querySelector('img');
        
        if (!imageElement) {
            imageElement = document.createElement('img');
            imageElement.alt = `Image from ${topic}`;
            imageElement.style.maxWidth = '100%';
            imageElement.style.height = 'auto';
            container.appendChild(imageElement);
        }
        
        // 处理不同的图像数据格式
        try {
            if (data.image_data) {
                // Base64编码的图像数据
                console.log(`显示图像，Base64长度: ${data.image_data.length}`);
                
                // 检查Base64数据是否需要添加前缀
                let base64Data = data.image_data;
                if (!base64Data.startsWith('data:')) {
                    // 根据数据类型添加适当的MIME类型
                    let mimeType = 'image/jpeg'; // 默认
                    
                    if (data.encoding === 'png' || base64Data.startsWith('iVBOR')) {
                        mimeType = 'image/png';
                    } else if (base64Data.startsWith('/9j/')) {
                        mimeType = 'image/jpeg';
                    } else if (base64Data.startsWith('R0lGOD')) {
                        mimeType = 'image/gif';
                    }
                    
                    base64Data = `data:${mimeType};base64,${base64Data}`;
                }
                
                imageElement.src = base64Data;
                
                // 添加加载事件监听
                imageElement.onload = () => {
                    console.log(`图像加载成功: ${imageElement.naturalWidth}x${imageElement.naturalHeight}`);
                };
                
                imageElement.onerror = (error) => {
                    console.error('图像加载失败:', error);
                    this.createPlaceholderImage(imageElement, `图像加载失败: ${topic}`);
                };
                
            } else if (data.data && typeof data.data === 'string') {
                // 直接的base64数据
                console.log(`显示直接Base64数据，长度: ${data.data.length}`);
                
                let base64Data = data.data;
                if (!base64Data.startsWith('data:')) {
                    base64Data = `data:image/jpeg;base64,${base64Data}`;
                }
                
                imageElement.src = base64Data;
                
            } else if (data.raw && data.raw instanceof ArrayBuffer) {
                // 二进制图像数据
                console.log('显示二进制图像数据');
                const blob = new Blob([data.raw], { type: 'image/jpeg' });
                const url = URL.createObjectURL(blob);
                imageElement.src = url;
                
                // 清理URL
                imageElement.onload = () => {
                    URL.revokeObjectURL(url);
                    console.log('二进制图像加载成功');
                };
            } else {
                // 创建占位图像
                console.log('创建占位图像，数据格式未识别');
                this.createPlaceholderImage(imageElement, `来自 ${topic} 的图像数据`);
            }
            
            // 应用自动调整大小设置
            if (this.autoResizeCheck && this.autoResizeCheck.checked) {
                imageElement.style.width = '100%';
                imageElement.style.height = 'auto';
            } else {
                imageElement.style.width = 'auto';
                imageElement.style.height = 'auto';
            }
            
        } catch (error) {
            console.error('显示图像时出错:', error);
            this.createPlaceholderImage(imageElement, `图像显示错误: ${error.message}`);
        }
    }

    displayVideo(container, data, topic) {
        let videoElement = container.querySelector('video');
        
        if (!videoElement) {
            videoElement = document.createElement('video');
            videoElement.controls = true;
            videoElement.autoplay = false;
            videoElement.muted = true;
            container.appendChild(videoElement);
        }
        
        // 处理视频流数据
        if (data.video_url) {
            videoElement.src = data.video_url;
        } else if (data.stream_data) {
            // WebRTC或流数据处理
            this.setupVideoStream(videoElement, data.stream_data);
        } else {
            // 创建占位视频
            videoElement.poster = 'data:image/svg+xml;base64,' + btoa(`
                <svg xmlns="http://www.w3.org/2000/svg" width="400" height="300" viewBox="0 0 400 300">
                    <rect width="400" height="300" fill="#f8f9fa"/>
                    <text x="200" y="150" font-family="Arial" font-size="18" text-anchor="middle" fill="#6c757d">
                        视频流: ${topic}
                    </text>
                </svg>
            `);
        }
    }

    displayDepthImage(container, data, topic) {
        let canvasElement = container.querySelector('canvas');
        
        if (!canvasElement) {
            canvasElement = document.createElement('canvas');
            canvasElement.width = 640;
            canvasElement.height = 480;
            container.appendChild(canvasElement);
        }
        
        const ctx = canvasElement.getContext('2d');
        
        // 处理深度图数据
        if (data.depth_data && data.width && data.height) {
            this.renderDepthData(ctx, data.depth_data, data.width, data.height);
        } else {
            // 创建占位深度图
            ctx.fillStyle = '#f8f9fa';
            ctx.fillRect(0, 0, canvasElement.width, canvasElement.height);
            ctx.fillStyle = '#6c757d';
            ctx.font = '18px Arial';
            ctx.textAlign = 'center';
            ctx.fillText(`深度图: ${topic}`, canvasElement.width/2, canvasElement.height/2);
        }
    }

    displayPointCloud(container, data, topic) {
        let canvasElement = container.querySelector('canvas');
        
        if (!canvasElement) {
            canvasElement = document.createElement('canvas');
            canvasElement.width = 640;
            canvasElement.height = 480;
            container.appendChild(canvasElement);
        }
        
        const ctx = canvasElement.getContext('2d');
        
        // 处理点云数据
        if (data.points && Array.isArray(data.points)) {
            this.renderPointCloud(ctx, data.points, canvasElement.width, canvasElement.height);
        } else {
            // 创建占位点云
            ctx.fillStyle = '#f8f9fa';
            ctx.fillRect(0, 0, canvasElement.width, canvasElement.height);
            ctx.fillStyle = '#6c757d';
            ctx.font = '18px Arial';
            ctx.textAlign = 'center';
            ctx.fillText(`点云: ${topic}`, canvasElement.width/2, canvasElement.height/2);
        }
    }

    displayRawData(container, data, topic) {
        const preElement = document.createElement('pre');
        preElement.style.cssText = `
            background: #f8f9fa;
            padding: 1rem;
            border-radius: 4px;
            font-size: 0.8rem;
            overflow: auto;
            max-height: 300px;
            white-space: pre-wrap;
        `;
        
        if (typeof data === 'object') {
            preElement.textContent = JSON.stringify(data, null, 2);
        } else {
            preElement.textContent = data.toString();
        }
        
        container.appendChild(preElement);
    }

    renderDepthData(ctx, depthData, width, height) {
        const imageData = ctx.createImageData(width, height);
        const data = imageData.data;
        
        // 将深度数据转换为灰度图像
        for (let i = 0; i < depthData.length; i++) {
            const depth = depthData[i];
            const normalized = Math.min(255, Math.max(0, depth * 255));
            
            const pixelIndex = i * 4;
            data[pixelIndex] = normalized;     // R
            data[pixelIndex + 1] = normalized; // G
            data[pixelIndex + 2] = normalized; // B
            data[pixelIndex + 3] = 255;        // A
        }
        
        ctx.putImageData(imageData, 0, 0);
    }

    renderPointCloud(ctx, points, width, height) {
        ctx.fillStyle = '#000000';
        ctx.fillRect(0, 0, width, height);
        
        // 简单的点云渲染
        ctx.fillStyle = '#ffffff';
        for (let i = 0; i < points.length; i += 3) {
            const x = (points[i] + 1) * width / 2;
            const y = (points[i + 1] + 1) * height / 2;
            
            if (x >= 0 && x < width && y >= 0 && y < height) {
                ctx.fillRect(Math.floor(x), Math.floor(y), 2, 2);
            }
        }
    }

    updateMediaTimestamp(topic) {
        if (!this.showTimestampCheck.checked) return;
        
        const timestampElement = document.getElementById(`timestamp-${this.sanitizeTopicId(topic)}`);
        if (timestampElement) {
            const now = new Date();
            timestampElement.textContent = now.toLocaleTimeString();
        }
    }

    updateMetadata(topic, data) {
        if (!this.showMetadataCheck.checked) return;
        
        const metadataElement = document.getElementById(`metadata-${this.sanitizeTopicId(topic)}`);
        if (!metadataElement) return;
        
        const metadata = this.extractMetadata(data);
        metadataElement.innerHTML = '';
        
        for (const [key, value] of Object.entries(metadata)) {
            const metadataItem = document.createElement('div');
            metadataItem.className = 'metadata-item';
            metadataItem.innerHTML = `
                <span>${key}:</span>
                <span>${value}</span>
            `;
            metadataElement.appendChild(metadataItem);
        }
    }

    extractMetadata(data) {
        const metadata = {};
        
        // 基本尺寸信息
        if (data.width && data.height) {
            metadata['尺寸'] = `${data.width}×${data.height}`;
        }
        
        // 编码格式
        if (data.encoding) {
            metadata['编码'] = data.encoding;
        }
        
        // 数据类型
        if (data.type) {
            metadata['类型'] = this.getTypeDisplayName(data.type);
        }
        
        // 帧率
        if (data.fps) {
            metadata['帧率'] = `${data.fps} fps`;
        }
        
        // 文件大小
        if (data.size) {
            metadata['大小'] = this.formatBytes(data.size);
        } else if (data.original_size) {
            metadata['大小'] = this.formatBytes(data.original_size);
        } else if (data.image_data) {
            // 估算Base64数据的原始大小
            const estimatedSize = Math.floor(data.image_data.length * 3 / 4);
            metadata['估算大小'] = this.formatBytes(estimatedSize);
        }
        
        // 时间戳
        if (data.timestamp) {
            const date = new Date(data.timestamp);
            metadata['时间'] = date.toLocaleTimeString();
        }
        
        // 数据源
        if (data.source) {
            metadata['来源'] = data.source;
        }
        
        // 坐标系
        if (data.frame_id) {
            metadata['坐标系'] = data.frame_id;
        }
        
        // 深度图特有信息
        if (data.min_depth !== undefined && data.max_depth !== undefined) {
            metadata['深度范围'] = `${data.min_depth.toFixed(2)} - ${data.max_depth.toFixed(2)}m`;
        }
        
        // 点云特有信息
        if (data.point_count) {
            metadata['点数'] = data.point_count.toLocaleString();
        }
        
        // 压缩信息
        if (data.compressed) {
            metadata['压缩'] = data.compressed ? '是' : '否';
        }
        
        return metadata;
    }

    updateStats() {
        this.subscribedCountSpan.textContent = this.subscribedTopics.size;
        this.messageCountSpan.textContent = this.messageCount;
        
        // 计算活跃流
        const now = Date.now();
        let activeStreams = 0;
        for (const [topic, info] of this.subscribedTopics) {
            if (info.lastMessage && (now - info.lastMessage) < 10000) {
                activeStreams++;
            }
        }
        this.activeStreamsSpan.textContent = activeStreams;
    }

    updateDataRate() {
        const now = Date.now();
        const timeDiff = (now - this.lastUpdateTime) / 1000;
        const dataRate = this.dataTransferSize / timeDiff;
        
        this.dataRateSpan.textContent = this.formatBytes(dataRate) + '/s';
        
        // 重置计数器
        this.dataTransferSize = 0;
        this.lastUpdateTime = now;
    }

    formatBytes(bytes) {
        if (bytes === 0) return '0 B';
        const k = 1024;
        const sizes = ['B', 'KB', 'MB', 'GB'];
        const i = Math.floor(Math.log(bytes) / Math.log(k));
        return parseFloat((bytes / Math.pow(k, i)).toFixed(1)) + ' ' + sizes[i];
    }

    toggleFullscreen() {
        // 这里可以实现整个应用的全屏模式
        if (document.fullscreenElement) {
            document.exitFullscreen();
        } else {
            document.documentElement.requestFullscreen();
        }
    }

    toggleMediaFullscreen(topic) {
        const container = document.getElementById(`media-${this.sanitizeTopicId(topic)}`);
        const mediaElement = container.querySelector('img, video, canvas');
        
        if (mediaElement) {
            this.showFullscreenViewer(mediaElement, topic);
        }
    }

    showFullscreenViewer(mediaElement, topic) {
        const fullscreenTitle = document.getElementById('fullscreenTitle');
        fullscreenTitle.textContent = `全屏查看: ${topic}`;
        
        // 隐藏所有全屏媒体元素
        this.fullscreenImage.style.display = 'none';
        this.fullscreenVideo.style.display = 'none';
        this.fullscreenCanvas.style.display = 'none';
        
        // 根据媒体类型显示相应元素
        if (mediaElement.tagName === 'IMG') {
            this.fullscreenImage.src = mediaElement.src;
            this.fullscreenImage.style.display = 'block';
        } else if (mediaElement.tagName === 'VIDEO') {
            this.fullscreenVideo.src = mediaElement.src;
            this.fullscreenVideo.style.display = 'block';
        } else if (mediaElement.tagName === 'CANVAS') {
            const ctx = this.fullscreenCanvas.getContext('2d');
            this.fullscreenCanvas.width = mediaElement.width;
            this.fullscreenCanvas.height = mediaElement.height;
            ctx.drawImage(mediaElement, 0, 0);
            this.fullscreenCanvas.style.display = 'block';
        }
        
        this.fullscreenViewer.style.display = 'flex';
    }

    exitFullscreen() {
        this.fullscreenViewer.style.display = 'none';
    }

    saveCurrentImages() {
        const images = document.querySelectorAll('.media-container img, .media-container canvas');
        
        images.forEach((element, index) => {
            const link = document.createElement('a');
            
            if (element.tagName === 'IMG') {
                link.href = element.src;
                link.download = `image_${index}_${Date.now()}.jpg`;
            } else if (element.tagName === 'CANVAS') {
                link.href = element.toDataURL();
                link.download = `canvas_${index}_${Date.now()}.png`;
            }
            
            document.body.appendChild(link);
            link.click();
            document.body.removeChild(link);
        });
        
        this.showNotification('图像已保存', 'success');
    }

    toggleRecording() {
        this.isRecording = !this.isRecording;
        
        if (this.isRecording) {
            this.recordBtn.textContent = '停止录制';
            this.recordBtn.style.background = 'var(--accent-color)';
            this.recordedFrames = [];
            this.showNotification('开始录制...', 'info');
        } else {
            this.recordBtn.textContent = '开始录制';
            this.recordBtn.style.background = 'var(--secondary-color)';
            this.exportRecording();
            this.showNotification('录制已停止', 'info');
        }
    }

    recordFrame(topic, data) {
        this.recordedFrames.push({
            topic,
            data,
            timestamp: Date.now()
        });
        
        // 限制录制帧数，避免内存溢出
        if (this.recordedFrames.length > 1000) {
            this.recordedFrames.shift();
        }
    }

    exportRecording() {
        if (this.recordedFrames.length === 0) return;
        
        const exportData = {
            recording: this.recordedFrames,
            startTime: this.recordedFrames[0].timestamp,
            endTime: this.recordedFrames[this.recordedFrames.length - 1].timestamp,
            frameCount: this.recordedFrames.length
        };
        
        const dataStr = JSON.stringify(exportData, null, 2);
        const dataBlob = new Blob([dataStr], { type: 'application/json' });
        
        const link = document.createElement('a');
        link.href = URL.createObjectURL(dataBlob);
        link.download = `recording_${Date.now()}.json`;
        document.body.appendChild(link);
        link.click();
        document.body.removeChild(link);
        
        URL.revokeObjectURL(link.href);
    }

    toggleConfig() {
        const isHidden = this.configContent.style.display === 'none';
        this.configContent.style.display = isHidden ? 'block' : 'none';
        this.toggleConfigBtn.textContent = isHidden ? '折叠' : '展开';
    }

    refreshMedia(topic) {
        // 重新请求该主题的最新数据
        if (this.mqttClient && this.isConnected) {
            // 发送刷新请求（如果服务器支持）
            const refreshTopic = `${topic}/refresh`;
            this.mqttClient.publish(refreshTopic, JSON.stringify({
                command: 'refresh',
                timestamp: Date.now()
            }));
        }
    }

    saveMediaContent(topic) {
        const container = document.getElementById(`media-${this.sanitizeTopicId(topic)}`);
        const mediaElement = container.querySelector('img, video, canvas');
        
        if (mediaElement) {
            const link = document.createElement('a');
            
            if (mediaElement.tagName === 'IMG') {
                link.href = mediaElement.src;
                link.download = `${this.sanitizeTopicId(topic)}_${Date.now()}.jpg`;
            } else if (mediaElement.tagName === 'CANVAS') {
                link.href = mediaElement.toDataURL();
                link.download = `${this.sanitizeTopicId(topic)}_${Date.now()}.png`;
            } else if (mediaElement.tagName === 'VIDEO') {
                // 对于视频，保存当前帧
                const canvas = document.createElement('canvas');
                canvas.width = mediaElement.videoWidth;
                canvas.height = mediaElement.videoHeight;
                const ctx = canvas.getContext('2d');
                ctx.drawImage(mediaElement, 0, 0);
                link.href = canvas.toDataURL();
                link.download = `${this.sanitizeTopicId(topic)}_frame_${Date.now()}.png`;
            }
            
            document.body.appendChild(link);
            link.click();
            document.body.removeChild(link);
            
            this.showNotification(`已保存 ${topic} 的媒体内容`, 'success');
        }
    }

    createPlaceholderImage(imgElement, altText) {
        const svg = `
            <svg xmlns="http://www.w3.org/2000/svg" width="400" height="300" viewBox="0 0 400 300">
                <rect width="400" height="300" fill="#f8f9fa" stroke="#dee2e6"/>
                <text x="200" y="140" font-family="Arial" font-size="14" text-anchor="middle" fill="#6c757d">
                    ${altText}
                </text>
                <text x="200" y="165" font-family="Arial" font-size="12" text-anchor="middle" fill="#adb5bd">
                    等待数据...
                </text>
            </svg>
        `;
        imgElement.src = 'data:image/svg+xml;base64,' + btoa(svg);
    }

    showNotification(message, type = 'info') {
        const container = document.getElementById('notificationContainer');
        const notification = document.createElement('div');
        notification.className = `notification ${type}`;
        
        const icons = {
            success: '✅',
            error: '❌',
            warning: '⚠️',
            info: 'ℹ️'
        };
        
        notification.innerHTML = `
            <div class="notification-icon">${icons[type] || icons.info}</div>
            <div class="notification-content">
                <div class="notification-message">${message}</div>
            </div>
            <button class="notification-close" onclick="this.parentElement.remove()">✕</button>
        `;
        
        container.appendChild(notification);
        
        // 自动移除通知
        setTimeout(() => {
            if (notification.parentElement) {
                notification.remove();
            }
        }, 5000);
    }

    getTypeIcon(type) {
        const icons = {
            image: '🖼️',
            video: '🎥',
            depth: '🕳️',
            pointcloud: '☁️'
        };
        return icons[type] || '📄';
    }

    getTypeDisplayName(type) {
        const names = {
            image: '图像',
            video: '视频',
            depth: '深度图',
            pointcloud: '点云'
        };
        return names[type] || type;
    }

    sanitizeTopicId(topic) {
        return topic.replace(/[^a-zA-Z0-9]/g, '_');
    }

    setupVideoStream(videoElement, streamData) {
        // 这里可以实现WebRTC或其他视频流的设置
        console.log('Setting up video stream:', streamData);
        
        // 示例：如果streamData包含WebRTC配置
        if (streamData.webrtc) {
            // 实现WebRTC连接
        } else if (streamData.hls) {
            // 实现HLS流
            if (videoElement.canPlayType('application/vnd.apple.mpegurl')) {
                videoElement.src = streamData.hls;
            }
        } else if (streamData.dash) {
            // 实现DASH流
        }
    }
}

// 初始化多媒体监控系统
let mediaMonitor;

document.addEventListener('DOMContentLoaded', () => {
    mediaMonitor = new MediaMonitor();
    console.log('ROS2 MQTT 多媒体监控系统已启动');
});
