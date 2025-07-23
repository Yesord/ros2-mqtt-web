// Apple Bento风格媒体监控器 - 专注Base64图像处理
class MediaMonitorBento {
    constructor() {
        this.client = null;
        this.isConnected = false;
        this.subscribedTopics = new Set();
        
        // Canvas上下文
        this.videoCanvas = null;
        this.videoCtx = null;
        this.fullscreenCanvas = null;
        this.fullscreenCtx = null;
        
        // 图像处理
        this.lastImage = null;
        this.frameBuffer = [];
        this.maxFrameBuffer = 30; // 保持30帧用于视频流
        this.autoPlay = true;
        this.fps = 0;
        this.lastFrameTime = 0;
        this.frameCount = 0;
        
        // 统计数据
        this.stats = {
            totalMessages: 0,
            totalImages: 0,
            dataTransferred: 0,
            sessionStartTime: Date.now()
        };
        
        // UI元素
        this.elements = {};
        
        this.init();
    }
    
    init() {
        this.initElements();
        this.initCanvas();
        this.initEventListeners();
        this.startStatsUpdate();
        this.showNotification('系统初始化完成', 'success');
    }
    
    initElements() {
        // 获取所有需要的DOM元素
        this.elements = {
            // 连接相关
            brokerHost: document.getElementById('brokerHost'),
            brokerPort: document.getElementById('brokerPort'),
            clientId: document.getElementById('clientId'),
            connectBtn: document.getElementById('connectBtn'),
            disconnectBtn: document.getElementById('disconnectBtn'),
            connectionStatus: document.getElementById('connectionStatus'),
            statusIndicator: document.querySelector('.status-indicator'),
            statusText: document.querySelector('.status-text'),
            
            // 话题相关
            topicInput: document.getElementById('topicInput'),
            addTopicBtn: document.getElementById('addTopicBtn'),
            activeTopicsList: document.getElementById('activeTopicsList'),
            
            // 视频流
            videoCanvas: document.getElementById('videoCanvas'),
            playBtn: document.getElementById('playBtn'),
            pauseBtn: document.getElementById('pauseBtn'),
            fullscreenBtn: document.getElementById('fullscreenBtn'),
            fpsDisplay: document.getElementById('fpsDisplay'),
            
            // 最新图像
            latestImage: document.getElementById('latestImage'),
            saveImageBtn: document.getElementById('saveImageBtn'),
            clearImageBtn: document.getElementById('clearImageBtn'),
            
            // 统计信息
            totalMessagesCount: document.getElementById('totalMessagesCount'),
            totalImagesCount: document.getElementById('totalImagesCount'),
            dataTransferredCount: document.getElementById('dataTransferredCount'),
            sessionTimeCount: document.getElementById('sessionTimeCount'),
            
            // 控制面板
            frameRateSlider: document.getElementById('frameRateSlider'),
            frameRateValue: document.getElementById('frameRateValue'),
            bufferSizeSlider: document.getElementById('bufferSizeSlider'),
            bufferSizeValue: document.getElementById('bufferSizeValue'),
            autoPlaySwitch: document.getElementById('autoPlaySwitch'),
            loopPlaybackSwitch: document.getElementById('loopPlaybackSwitch'),
            
            // 模态框
            fullscreenModal: document.getElementById('fullscreenModal'),
            fullscreenCanvas: document.getElementById('fullscreenCanvas'),
            closeModalBtn: document.getElementById('closeModalBtn')
        };
    }
    
    initCanvas() {
        // 初始化视频Canvas
        this.videoCanvas = this.elements.videoCanvas;
        this.videoCtx = this.videoCanvas.getContext('2d');
        
        // 初始化全屏Canvas
        this.fullscreenCanvas = this.elements.fullscreenCanvas;
        this.fullscreenCtx = this.fullscreenCanvas.getContext('2d');
        
        // 设置默认尺寸
        this.resizeCanvas();
        window.addEventListener('resize', () => this.resizeCanvas());
    }
    
    resizeCanvas() {
        // 视频Canvas自适应容器
        const container = this.videoCanvas.parentElement;
        const rect = container.getBoundingClientRect();
        this.videoCanvas.width = rect.width;
        this.videoCanvas.height = rect.height;
        
        // 全屏Canvas设置为窗口大小
        this.fullscreenCanvas.width = window.innerWidth;
        this.fullscreenCanvas.height = window.innerHeight;
    }
    
    initEventListeners() {
        // 连接相关事件
        this.elements.connectBtn.addEventListener('click', () => this.connect());
        this.elements.disconnectBtn.addEventListener('click', () => this.disconnect());
        
        // 话题相关事件
        this.elements.addTopicBtn.addEventListener('click', () => this.addTopic());
        this.elements.topicInput.addEventListener('keypress', (e) => {
            if (e.key === 'Enter') this.addTopic();
        });
        
        // 预设话题按钮
        document.querySelectorAll('.preset-topic-btn').forEach(btn => {
            btn.addEventListener('click', (e) => {
                const topic = e.target.getAttribute('data-topic') || e.target.textContent.trim();
                this.elements.topicInput.value = topic;
                this.addTopic();
            });
        });
        
        // 视频控制事件
        this.elements.playBtn.addEventListener('click', () => this.playVideo());
        this.elements.pauseBtn.addEventListener('click', () => this.pauseVideo());
        this.elements.fullscreenBtn.addEventListener('click', () => this.openFullscreen());
        
        // 图像控制事件
        this.elements.saveImageBtn.addEventListener('click', () => this.saveCurrentImage());
        this.elements.clearImageBtn.addEventListener('click', () => this.clearImage());
        
        // 控制面板事件
        this.elements.frameRateSlider.addEventListener('input', (e) => {
            this.elements.frameRateValue.textContent = e.target.value + ' FPS';
        });
        
        this.elements.bufferSizeSlider.addEventListener('input', (e) => {
            const value = parseInt(e.target.value);
            this.maxFrameBuffer = value;
            this.elements.bufferSizeValue.textContent = value + ' 帧';
            // 调整当前缓冲区大小
            if (this.frameBuffer.length > value) {
                this.frameBuffer = this.frameBuffer.slice(-value);
            }
        });
        
        this.elements.autoPlaySwitch.addEventListener('change', (e) => {
            this.autoPlay = e.target.checked;
            if (this.autoPlay && this.frameBuffer.length > 0) {
                this.playVideo();
            }
        });
        
        // 模态框事件
        this.elements.closeModalBtn.addEventListener('click', () => this.closeFullscreen());
        this.elements.fullscreenModal.addEventListener('click', (e) => {
            if (e.target === this.elements.fullscreenModal) {
                this.closeFullscreen();
            }
        });
        
        // 键盘事件
        document.addEventListener('keydown', (e) => {
            if (e.key === 'Escape') {
                this.closeFullscreen();
            }
        });
    }
    
    connect() {
        const host = this.elements.brokerHost.value.trim();
        const port = parseInt(this.elements.brokerPort.value);
        const clientId = this.elements.clientId.value.trim();
        
        if (!host || !port || !clientId) {
            this.showNotification('请填充完整的连接信息', 'error');
            return;
        }
        
        try {
            this.updateConnectionStatus('connecting', '连接中...');
            
            // 检查mqtt.js是否可用
            if (typeof mqtt === 'undefined') {
                throw new Error('MQTT.js 库未加载');
            }
            
            // 使用WebSocket连接MQTT
            const wsUrl = `ws://${host}:${port}`;
            this.client = mqtt.connect(wsUrl, {
                clientId: clientId,
                keepalive: 60,
                clean: true,
                reconnectPeriod: 1000,
                connectTimeout: 30 * 1000,
            });
            
            // 设置事件监听器
            this.client.on('connect', () => {
                this.onConnected();
            });
            
            this.client.on('error', (error) => {
                this.onConnectionFailed(error);
            });
            
            this.client.on('close', () => {
                this.onConnectionLost({ errorMessage: '连接已关闭' });
            });
            
            this.client.on('message', (topic, message) => {
                this.onMessageReceived({
                    destinationName: topic,
                    payloadString: message.toString(),
                    payloadBytes: message
                });
            });
            
        } catch (error) {
            console.error('连接错误:', error);
            this.showNotification('连接失败: ' + error.message, 'error');
            this.updateConnectionStatus('disconnected', '连接失败');
        }
    }
    
    disconnect() {
        if (this.client && this.client.connected) {
            this.client.end();
        }
        this.onDisconnected();
    }
    
    onConnected() {
        this.isConnected = true;
        this.updateConnectionStatus('connected', '已连接');
        this.elements.connectBtn.disabled = true;
        this.elements.disconnectBtn.disabled = false;
        this.showNotification('MQTT连接成功', 'success');
    }
    
    onConnectionFailed(error) {
        console.error('连接失败:', error);
        this.updateConnectionStatus('disconnected', '连接失败');
        this.showNotification('连接失败: ' + (error.errorMessage || '未知错误'), 'error');
    }
    
    onConnectionLost(responseObject) {
        this.isConnected = false;
        this.updateConnectionStatus('disconnected', '连接丢失');
        this.elements.connectBtn.disabled = false;
        this.elements.disconnectBtn.disabled = true;
        
        if (responseObject.errorCode !== 0) {
            this.showNotification('连接丢失: ' + responseObject.errorMessage, 'warning');
        }
    }
    
    onDisconnected() {
        this.isConnected = false;
        this.updateConnectionStatus('disconnected', '已断开');
        this.elements.connectBtn.disabled = false;
        this.elements.disconnectBtn.disabled = true;
        this.subscribedTopics.clear();
        this.updateActiveTopicsList();
        this.showNotification('已断开MQTT连接', 'success');
    }
    
    updateConnectionStatus(status, text) {
        this.elements.statusIndicator.className = `status-indicator ${status}`;
        this.elements.statusText.textContent = text;
    }
    
    addTopic() {
        const topic = this.elements.topicInput.value.trim();
        if (!topic) {
            this.showNotification('请输入话题名称', 'warning');
            return;
        }
        
        if (!this.isConnected) {
            this.showNotification('请先连接MQTT服务器', 'warning');
            return;
        }
        
        if (this.subscribedTopics.has(topic)) {
            this.showNotification('话题已订阅', 'warning');
            return;
        }
        
        try {
            this.client.subscribe(topic);
            this.subscribedTopics.add(topic);
            this.elements.topicInput.value = '';
            this.updateActiveTopicsList();
            this.showNotification(`已订阅话题: ${topic}`, 'success');
        } catch (error) {
            console.error('订阅失败:', error);
            this.showNotification('订阅失败: ' + error.message, 'error');
        }
    }
    
    removeTopic(topic) {
        if (!this.isConnected) return;
        
        try {
            this.client.unsubscribe(topic);
            this.subscribedTopics.delete(topic);
            this.updateActiveTopicsList();
            this.showNotification(`已取消订阅: ${topic}`, 'success');
        } catch (error) {
            console.error('取消订阅失败:', error);
            this.showNotification('取消订阅失败: ' + error.message, 'error');
        }
    }
    
    updateActiveTopicsList() {
        const container = this.elements.activeTopicsList;
        container.innerHTML = '';
        
        if (this.subscribedTopics.size === 0) {
            container.innerHTML = '<div style="text-align: center; color: var(--text-secondary); font-size: 12px;">暂无订阅话题</div>';
            return;
        }
        
        this.subscribedTopics.forEach(topic => {
            const item = document.createElement('div');
            item.className = 'active-topic-item';
            item.innerHTML = `
                <span class="topic-name">${topic}</span>
                <span class="topic-status">活跃</span>
                <button class="remove-topic-btn" onclick="mediaMonitor.removeTopic('${topic}')">×</button>
            `;
            container.appendChild(item);
        });
    }
    
    onMessageReceived(message) {
        try {
            this.stats.totalMessages++;
            this.stats.dataTransferred += message.payloadBytes ? message.payloadBytes.length : message.payloadString.length;
            
            const topic = message.destinationName;
            const payload = message.payloadString;
            
            console.log('收到MQTT消息:', {
                topic: topic,
                payloadLength: payload.length,
                payloadPreview: payload.substring(0, 100) + (payload.length > 100 ? '...' : '')
            });
            
            // 解析JSON消息
            const messageData = this.parseMessage(payload);
            if (messageData && messageData.imageData) {
                console.log('解析到图像数据:', {
                    imageDataType: typeof messageData.imageData,
                    imageDataLength: messageData.imageData.length,
                    metadata: messageData.metadata
                });
                this.handleImageData(messageData.imageData, messageData.metadata);
            } else {
                console.warn('无法解析图像数据，消息格式:', typeof payload);
                console.log('原始消息内容:', payload.substring(0, 200));
            }
            
        } catch (error) {
            console.error('消息处理失败:', error);
        }
    }
    
    parseMessage(payload) {
        try {
            // 尝试解析JSON
            const jsonData = JSON.parse(payload);
            
            console.log('JSON解析成功，检查数据格式:', Object.keys(jsonData));
            
            // 检查不同的数据格式（参考mqtt_image_receiver.py逻辑）
            if (jsonData.data) {
                const dataField = jsonData.data;
                
                if (typeof dataField === 'object' && dataField !== null) {
                    // 检查MQTT接口的自定义序列化格式
                    if (dataField.type === 'uint8_array' && dataField.data && dataField.size) {
                        console.log('识别为uint8_array格式（MQTT格式）');
                        return {
                            imageData: dataField, // 保持完整对象结构
                            metadata: {
                                timestamp: jsonData.timestamp || Date.now(),
                                encoding: 'base64',
                                format: 'jpeg',
                                source: jsonData.source,
                                source_topic: jsonData.source_topic,
                                originalSize: dataField.size
                            }
                        };
                    } else if (dataField.encoding === 'base64' && dataField.data) {
                        console.log('识别为新格式（包含encoding字段）');
                        return {
                            imageData: dataField,
                            metadata: {
                                timestamp: jsonData.timestamp || Date.now(),
                                encoding: dataField.encoding,
                                format: jsonData.format || 'jpeg',
                                dataType: dataField.data_type
                            }
                        };
                    }
                } else if (typeof dataField === 'string' && dataField.length > 100) {
                    console.log('识别为直接Base64字符串');
                    return {
                        imageData: dataField,
                        metadata: {
                            timestamp: jsonData.timestamp || Date.now(),
                            encoding: 'base64',
                            format: jsonData.format || 'jpeg',
                            source: jsonData.source,
                            source_topic: jsonData.source_topic
                        }
                    };
                }
            } else if (jsonData.image_data) {
                // 格式2: {image_data: "base64string", ...}
                console.log('识别为image_data格式');
                return {
                    imageData: jsonData.image_data,
                    metadata: {
                        timestamp: jsonData.timestamp || Date.now(),
                        encoding: jsonData.encoding || 'base64',
                        format: jsonData.format || 'jpeg'
                    }
                };
            } else if (jsonData.type === 'uint8_array' && jsonData.data) {
                // 格式3: 直接的图像数据对象 {type: 'uint8_array', size: xxx, data: 'base64'}
                console.log('识别为直接uint8_array对象');
                return {
                    imageData: jsonData,
                    metadata: {
                        timestamp: Date.now(),
                        encoding: 'base64',
                        format: 'jpeg',
                        originalSize: jsonData.size
                    }
                };
            } else if (typeof jsonData === 'string' && jsonData.length > 100) {
                // 格式4: 直接的base64字符串
                console.log('识别为直接JSON字符串');
                return {
                    imageData: jsonData,
                    metadata: {
                        timestamp: Date.now(),
                        encoding: 'base64',
                        format: 'jpeg'
                    }
                };
            }
            
            console.warn('未识别的JSON数据格式:', {
                keys: Object.keys(jsonData),
                dataType: typeof jsonData.data,
                preview: JSON.stringify(jsonData).substring(0, 200)
            });
            
        } catch (e) {
            console.warn('JSON解析失败:', e.message);
            // 如果不是JSON，尝试作为直接的base64字符串
            if (typeof payload === 'string' && payload.length > 100) {
                console.log('尝试作为直接Base64字符串处理');
                return {
                    imageData: payload,
                    metadata: {
                        timestamp: Date.now(),
                        encoding: 'base64',
                        format: 'jpeg'
                    }
                };
            }
        }
        
        return null;
    }
    
    handleImageData(imageData, metadata = {}) {
        try {
            // 处理不同格式的图像数据
            let base64Data = imageData;
            
            if (typeof base64Data !== 'string') {
                console.log('imageData不是字符串类型:', typeof base64Data, base64Data);
                
                // 检查是否是包含data字段的对象（参考mqtt_image_receiver.py处理逻辑）
                if (base64Data && typeof base64Data === 'object') {
                    if (base64Data.type === 'uint8_array' && base64Data.data) {
                        // MQTT接口的自定义序列化格式 {type: 'uint8_array', size: xxx, data: 'base64'}
                        console.log('检测到uint8_array格式:', {
                            type: base64Data.type,
                            size: base64Data.size,
                            dataLength: base64Data.data.length
                        });
                        base64Data = base64Data.data;
                    } else if (base64Data.encoding === 'base64' && base64Data.data) {
                        // 新格式：包含encoding字段
                        console.log('检测到encoding格式:', base64Data.encoding);
                        base64Data = base64Data.data;
                    } else if (base64Data.data) {
                        // 通用data字段
                        console.log('检测到通用data字段格式');
                        base64Data = base64Data.data;
                    } else {
                        console.error('未识别的对象格式:', Object.keys(base64Data));
                        return;
                    }
                } else if (Array.isArray(base64Data)) {
                    // 如果是数组，尝试转换为Base64字符串
                    console.log('检测到数组格式，转换为Base64');
                    const uint8Array = new Uint8Array(base64Data);
                    base64Data = btoa(String.fromCharCode.apply(null, uint8Array));
                } else {
                    base64Data = String(base64Data);
                }
            }
            
            // 再次检查转换后的数据是否为字符串
            if (typeof base64Data !== 'string') {
                console.error('无法将imageData转换为字符串:', typeof base64Data);
                return;
            }
            
            // 验证Base64数据格式
            if (base64Data.length < 100) {
                console.error('Base64数据长度太短:', base64Data.length);
                return;
            }
            
            // 验证Base64格式
            if (!this.isValidBase64(base64Data)) {
                console.error('Base64数据格式无效');
                console.error('数据预览:', base64Data.substring(0, 100));
                return;
            }
            
            // 清理base64字符串
            if (base64Data.includes(',')) {
                console.log('检测到data URL格式，提取base64部分');
                base64Data = base64Data.split(',')[1];
            }
            
            // 移除可能的换行符和空格
            base64Data = base64Data.replace(/[\r\n\s]/g, '');
            
            console.log('最终base64数据:', {
                type: typeof base64Data,
                length: base64Data.length,
                isValid: this.isValidBase64(base64Data),
                preview: base64Data.substring(0, 50) + '...'
            });
            
            // 创建图像对象
            const img = new Image();
            img.onload = () => {
                console.log('图像加载成功:', img.width + 'x' + img.height);
                this.stats.totalImages++;
                
                // 添加到帧缓冲区
                const frameData = {
                    image: img,
                    timestamp: metadata.timestamp || Date.now(),
                    metadata: metadata
                };
                
                this.addFrame(frameData);
                
                // 更新最新图像显示
                this.updateLatestImage(img, metadata);
                
                // 如果自动播放启用，更新视频流
                if (this.autoPlay) {
                    this.updateVideoStream();
                }
                
                // 计算FPS
                this.calculateFPS();
            };
            
            img.onerror = (error) => {
                console.error('图像加载失败:', error);
                console.error('尝试加载的URL长度:', img.src.length);
                console.error('URL前缀:', img.src.substring(0, 100));
            };
            
            // 设置图像源
            const mimeType = this.getMimeType(metadata.format || 'jpeg');
            img.src = `data:${mimeType};base64,${base64Data}`;
            
        } catch (error) {
            console.error('图像数据处理失败:', error);
        }
    }
    
    getMimeType(format) {
        const mimeTypes = {
            'jpeg': 'image/jpeg',
            'jpg': 'image/jpeg',
            'png': 'image/png',
            'gif': 'image/gif',
            'webp': 'image/webp',
            'bmp': 'image/bmp'
        };
        return mimeTypes[format.toLowerCase()] || 'image/jpeg';
    }
    
    isValidBase64(str) {
        try {
            // 基本格式检查
            if (typeof str !== 'string' || str.length === 0) {
                return false;
            }
            
            // 移除可能的换行符和空格
            const cleanStr = str.replace(/[\r\n\s]/g, '');
            
            // 检查Base64字符集
            const base64Regex = /^[A-Za-z0-9+/]*={0,2}$/;
            if (!base64Regex.test(cleanStr)) {
                return false;
            }
            
            // 检查长度（Base64编码的长度应该是4的倍数）
            if (cleanStr.length % 4 !== 0) {
                return false;
            }
            
            // 尝试解码测试
            try {
                atob(cleanStr.substring(0, Math.min(100, cleanStr.length)));
                return true;
            } catch (e) {
                return false;
            }
        } catch (e) {
            return false;
        }
    }
    
    addFrame(frameData) {
        this.frameBuffer.push(frameData);
        
        // 保持缓冲区大小限制
        if (this.frameBuffer.length > this.maxFrameBuffer) {
            this.frameBuffer.shift();
        }
    }
    
    updateLatestImage(img, metadata) {
        // 更新最新图像显示
        const canvas = document.createElement('canvas');
        const ctx = canvas.getContext('2d');
        
        canvas.width = img.width;
        canvas.height = img.height;
        ctx.drawImage(img, 0, 0);
        
        this.elements.latestImage.src = canvas.toDataURL();
        this.lastImage = img;
        
        // 更新图像信息
        const imageInfo = this.elements.latestImage.parentElement.querySelector('.image-info');
        if (imageInfo) {
            const timestamp = new Date(metadata.timestamp || Date.now()).toLocaleTimeString();
            imageInfo.innerHTML = `
                <span>时间: ${timestamp}</span>
                <span>尺寸: ${img.width}x${img.height}</span>
            `;
        }
    }
    
    updateVideoStream() {
        if (this.frameBuffer.length === 0) return;
        
        const latestFrame = this.frameBuffer[this.frameBuffer.length - 1];
        this.drawImageToCanvas(latestFrame.image, this.videoCtx, this.videoCanvas);
        
        // 如果全屏模式打开，也更新全屏画布
        if (this.elements.fullscreenModal.style.display === 'flex') {
            this.drawImageToCanvas(latestFrame.image, this.fullscreenCtx, this.fullscreenCanvas);
        }
        
        // 隐藏无信号覆盖层
        const overlay = document.querySelector('.video-overlay');
        if (overlay) {
            overlay.classList.add('hidden');
        }
    }
    
    drawImageToCanvas(img, ctx, canvas) {
        // 计算适合canvas的缩放比例
        const canvasRatio = canvas.width / canvas.height;
        const imageRatio = img.width / img.height;
        
        let drawWidth, drawHeight, offsetX, offsetY;
        
        if (imageRatio > canvasRatio) {
            // 图像更宽，以宽度为准
            drawWidth = canvas.width;
            drawHeight = canvas.width / imageRatio;
            offsetX = 0;
            offsetY = (canvas.height - drawHeight) / 2;
        } else {
            // 图像更高，以高度为准
            drawHeight = canvas.height;
            drawWidth = canvas.height * imageRatio;
            offsetX = (canvas.width - drawWidth) / 2;
            offsetY = 0;
        }
        
        // 清空画布
        ctx.fillStyle = '#000';
        ctx.fillRect(0, 0, canvas.width, canvas.height);
        
        // 绘制图像
        ctx.drawImage(img, offsetX, offsetY, drawWidth, drawHeight);
    }
    
    calculateFPS() {
        const now = Date.now();
        this.frameCount++;
        
        if (now - this.lastFrameTime >= 1000) {
            this.fps = Math.round(this.frameCount * 1000 / (now - this.lastFrameTime));
            this.elements.fpsDisplay.textContent = this.fps + ' FPS';
            this.frameCount = 0;
            this.lastFrameTime = now;
        }
    }
    
    playVideo() {
        this.autoPlay = true;
        this.elements.autoPlaySwitch.checked = true;
        if (this.frameBuffer.length > 0) {
            this.updateVideoStream();
        }
    }
    
    pauseVideo() {
        this.autoPlay = false;
        this.elements.autoPlaySwitch.checked = false;
    }
    
    openFullscreen() {
        if (this.lastImage) {
            this.elements.fullscreenModal.style.display = 'flex';
            this.drawImageToCanvas(this.lastImage, this.fullscreenCtx, this.fullscreenCanvas);
        }
    }
    
    closeFullscreen() {
        this.elements.fullscreenModal.style.display = 'none';
    }
    
    saveCurrentImage() {
        if (!this.lastImage) {
            this.showNotification('没有可保存的图像', 'warning');
            return;
        }
        
        try {
            const canvas = document.createElement('canvas');
            const ctx = canvas.getContext('2d');
            canvas.width = this.lastImage.width;
            canvas.height = this.lastImage.height;
            ctx.drawImage(this.lastImage, 0, 0);
            
            const link = document.createElement('a');
            link.download = `image_${Date.now()}.png`;
            link.href = canvas.toDataURL();
            link.click();
            
            this.showNotification('图像已保存', 'success');
        } catch (error) {
            console.error('保存失败:', error);
            this.showNotification('保存失败', 'error');
        }
    }
    
    clearImage() {
        this.elements.latestImage.src = '';
        this.lastImage = null;
        this.frameBuffer = [];
        
        // 清空视频Canvas
        this.videoCtx.fillStyle = '#000';
        this.videoCtx.fillRect(0, 0, this.videoCanvas.width, this.videoCanvas.height);
        
        // 显示无信号覆盖层
        const overlay = document.querySelector('.video-overlay');
        if (overlay) {
            overlay.classList.remove('hidden');
        }
        
        this.showNotification('已清空图像缓存', 'success');
    }
    
    startStatsUpdate() {
        setInterval(() => {
            this.updateStats();
        }, 1000);
    }
    
    updateStats() {
        // 更新统计显示
        this.elements.totalMessagesCount.textContent = this.stats.totalMessages.toLocaleString();
        this.elements.totalImagesCount.textContent = this.stats.totalImages.toLocaleString();
        
        // 数据传输量转换
        const mb = this.stats.dataTransferred / (1024 * 1024);
        this.elements.dataTransferredCount.textContent = mb.toFixed(2) + ' MB';
        
        // 会话时间
        const sessionTime = Math.floor((Date.now() - this.stats.sessionStartTime) / 1000);
        const hours = Math.floor(sessionTime / 3600);
        const minutes = Math.floor((sessionTime % 3600) / 60);
        const seconds = sessionTime % 60;
        this.elements.sessionTimeCount.textContent = 
            `${hours.toString().padStart(2, '0')}:${minutes.toString().padStart(2, '0')}:${seconds.toString().padStart(2, '0')}`;
    }
    
    showNotification(message, type = 'info') {
        const container = document.querySelector('.notifications-container') || this.createNotificationContainer();
        
        const notification = document.createElement('div');
        notification.className = `notification ${type}`;
        notification.innerHTML = `
            <span>${message}</span>
        `;
        
        container.appendChild(notification);
        
        // 自动移除通知
        setTimeout(() => {
            notification.remove();
        }, 4000);
    }
    
    createNotificationContainer() {
        const container = document.createElement('div');
        container.className = 'notifications-container';
        document.body.appendChild(container);
        return container;
    }
}

// 初始化应用
let mediaMonitor;
document.addEventListener('DOMContentLoaded', () => {
    mediaMonitor = new MediaMonitorBento();
});
