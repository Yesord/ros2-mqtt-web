/**
 * GPS监控系统JavaScript模块
 * 处理MQTT数据接收和地图显示
 */

class GPSMonitor {
    constructor() {
        // 地图相关
        this.map = null;
        this.marker = null;
        this.polyline = null;
        this.path = [];
        this.showTrail = true;
        
        // GPS数据
        this.currentPosition = null;
        this.lastPosition = null;
        this.totalDistance = 0;
        this.messageCount = 0;
        
        // 桥接器统计
        this.bridgeStats = {
            messageCount: 0,
            uptimeSeconds: 0,
            bridgeName: '',
            lastMessageId: 0
        };
        
        // 连接状态
        this.wsConnected = false;
        this.gpsDataReceived = false;
        this.lastMessageTime = null;
        this.mqttConnected = false;
        this.subscriptionCompleted = false;
        
        // MQTT客户端
        this.client = null;
        this.reconnectAttempts = 0;
        this.maxReconnectAttempts = 5;
        
        // 数据质量评估
        this.dataQuality = 'unknown';
        this.messageHistory = [];
        this.maxHistorySize = 50;
        
        this.initMap();
        this.initMQTTConnection();
        this.setupEventListeners();
        this.startPeriodicTasks();
    }

    /**
     * 初始化高德地图
     */
    initMap() {
        try {
            // 检查高德地图API是否加载
            if (typeof AMap === 'undefined') {
                this.showMessage('高德地图API未加载，请检查网络连接或将HTML中的YOUR_AMAP_KEY替换为您的API密钥', 'error');
                return;
            }

            // 初始化地图 - 综保区级别缩放
            this.map = new AMap.Map('mapContainer', {
                zoom: 17, // 综保区级别缩放
                center: [113.8750345928, 22.5045523128],
                mapStyle: 'amap://styles/normal',
                features: ['bg', 'road', 'building', 'point'],
                viewMode: '2D',
                zooms: [15, 19],
                resizeEnable: true,
                rotateEnable: false,
                pitchEnable: false,
                scrollWheel: true,
                doubleClickZoom: true,
                dragEnable: true
            });

            // 创建GPS位置标记
            this.marker = new AMap.Marker({
                position: [113.8750345928, 22.5045523128],
                icon: this.createGPSIcon(),
                title: 'GPS实时位置',
                anchor: 'center',
                zIndex: 100
            });

            this.map.add(this.marker);

            // 创建轨迹线
            this.polyline = new AMap.Polyline({
                path: [],
                strokeColor: '#FF6B35',
                strokeWeight: 5,
                strokeOpacity: 0.8,
                strokeStyle: 'solid',
                lineJoin: 'round',
                lineCap: 'round',
                zIndex: 10
            });

            this.map.add(this.polyline);

            // 异步加载控件插件
            this.loadMapControls();

            this.showMessage('地图初始化完成', 'success');
        } catch (error) {
            console.error('地图初始化失败:', error);
            this.showMessage(`地图初始化失败: ${error.message}`, 'error');
        }
    }

    /**
     * 加载地图控件
     */
    loadMapControls() {
        setTimeout(() => {
            try {
                AMap.plugin(['AMap.ToolBar', 'AMap.Scale'], () => {
                    try {
                        const toolbar = new AMap.ToolBar({
                            position: 'RB'
                        });
                        this.map.addControl(toolbar);

                        const scale = new AMap.Scale({
                            position: 'LB'
                        });
                        this.map.addControl(scale);

                        console.log('地图控件加载完成');
                    } catch (controlError) {
                        console.warn('地图控件加载失败:', controlError);
                    }
                });
            } catch (pluginError) {
                console.warn('地图插件加载失败:', pluginError);
            }
        }, 100);
    }

    /**
     * 创建GPS图标
     */
    createGPSIcon() {
        return new AMap.Icon({
            size: new AMap.Size(32, 32),
            image: 'data:image/svg+xml;base64,' + btoa(`
                <svg xmlns="http://www.w3.org/2000/svg" width="32" height="32" viewBox="0 0 32 32">
                    <circle cx="16" cy="17" r="14" fill="rgba(0,0,0,0.1)" opacity="0.4"/>
                    <path d="M16 4c5.5 0 10 4.5 10 10 0 6.5-10 14-10 14s-10-7.5-10-14c0-5.5 4.5-10 10-10z" fill="#FF6B35" stroke="white" stroke-width="2"/>
                    <circle cx="16" cy="14" r="4" fill="white"/>
                    <circle cx="16" cy="14" r="2" fill="#FF6B35"/>
                </svg>
            `),
            imageSize: new AMap.Size(32, 32),
            imageOffset: new AMap.Pixel(0, 0)
        });
    }

    /**
     * 初始化MQTT连接
     */
    initMQTTConnection() {
        try {
            // 检查MQTT.js库是否已加载
            if (typeof mqtt === 'undefined') {
                console.error('MQTT.js库未加载');
                this.showMessage('MQTT.js库未加载，请检查网络连接', 'error');
                return;
            }

            this.connectToMQTTBroker();
            
        } catch (error) {
            console.error('MQTT初始化失败:', error);
            this.showMessage('MQTT初始化失败: ' + error.message, 'error');
        }
    }

    /**
     * 连接到MQTT服务器
     */
    connectToMQTTBroker() {
        try {
            const wsUrl = 'ws://localhost:9001';
            console.log('连接到MQTT服务器:', wsUrl);
            
            this.client = mqtt.connect(wsUrl, {
                clientId: `gps_monitor_${Math.random().toString(16).substr(2, 8)}`,
                keepalive: 60,
                reconnectPeriod: 1000,
                connectTimeout: 30000
            });

            this.setupMQTTEventHandlers();

        } catch (error) {
            console.error('MQTT连接失败:', error);
            this.showMessage('MQTT连接失败: ' + error.message, 'error');
        }
    }

    /**
     * 设置MQTT事件处理器
     */
    setupMQTTEventHandlers() {
        // 连接成功
        this.client.on('connect', () => {
            console.log('✓ MQTT连接成功');
            this.wsConnected = true;
            this.mqttConnected = true;
            this.reconnectAttempts = 0;
            this.updateConnectionStatus();
            this.showMessage('MQTT连接成功', 'success');
            
            // 订阅GPS话题
            this.subscribeToGPSTopics();
        });

        // 连接错误
        this.client.on('error', (error) => {
            console.error('MQTT连接错误:', error);
            this.wsConnected = false;
            this.mqttConnected = false;
            this.updateConnectionStatus();
            this.showMessage('MQTT连接错误: ' + error.message, 'error');
        });

        // 连接关闭
        this.client.on('close', () => {
            console.log('MQTT连接关闭');
            this.wsConnected = false;
            this.mqttConnected = false;
            this.updateConnectionStatus();
            
            if (this.reconnectAttempts < this.maxReconnectAttempts) {
                this.reconnectAttempts++;
                const delay = Math.min(1000 * Math.pow(2, this.reconnectAttempts), 30000);
                this.showMessage(`连接断开，${delay/1000}秒后重连... (${this.reconnectAttempts}/${this.maxReconnectAttempts})`, 'warning');
            } else {
                this.showMessage('MQTT连接失败，已停止重连', 'error');
            }
        });

        // 接收消息
        this.client.on('message', (topic, message) => {
            try {
                console.log('收到MQTT消息 - 话题:', topic);
                
                const payload = message.toString();
                
                // 检查是否是GPS相关话题
                if (topic.includes('gps') || topic.includes('GPS')) {
                    try {
                        const gpsData = JSON.parse(payload);
                        this.handleGPSMessage({
                            topic: topic,
                            payload: gpsData
                        });
                    } catch (e) {
                        console.warn('GPS消息JSON解析失败:', e);
                    }
                }
            } catch (error) {
                console.error('处理MQTT消息失败:', error);
            }
        });
    }

    /**
     * 订阅GPS话题
     */
    subscribeToGPSTopics() {
        try {
            const gpsTopics = ['ros2/gps/fix'];

            gpsTopics.forEach((topic) => {
                this.client.subscribe(topic, { qos: 0 }, (error) => {
                    if (error) {
                        console.error('订阅话题失败:', topic, error);
                        this.showMessage(`GPS话题订阅失败: ${topic}`, 'error');
                    } else {
                        console.log('✓ 成功订阅话题:', topic);
                        this.subscriptionCompleted = true;
                        this.showMessage(`GPS话题订阅成功: ${topic}`, 'success');
                    }
                });
            });
            
        } catch (error) {
            console.error('订阅GPS话题失败:', error);
            this.showMessage('GPS话题订阅失败', 'error');
        }
    }

    /**
     * 处理GPS消息
     */
    handleGPSMessage(data) {
        try {
            console.log('处理GPS消息:', data);
            
            const payload = data.payload;
            
            if (!payload) {
                console.warn('载荷为空');
                return;
            }
            
            let latitude = null;
            let longitude = null;
            
            // 支持多种数据格式
            if (payload.data && 
                typeof payload.data.latitude === 'number' && 
                typeof payload.data.longitude === 'number') {
                latitude = payload.data.latitude;
                longitude = payload.data.longitude;
            } else if (typeof payload.latitude === 'number' && 
                       typeof payload.longitude === 'number') {
                latitude = payload.latitude;
                longitude = payload.longitude;
            } else {
                console.warn('无效的GPS数据格式:', payload);
                return;
            }

            // 保存上一个位置
            this.lastPosition = this.currentPosition;
            
            // 更新当前位置
            this.currentPosition = {
                lat: latitude,
                lng: longitude,
                timestamp: new Date(payload.timestamp || new Date().toISOString()),
                messageId: payload.message_id || 0,
                sourceNode: payload.source_node || 'unknown',
                sourceTopic: payload.source_topic || data.topic || 'unknown'
            };

            // 更新桥接器统计信息
            if (payload.bridge_name) {
                this.bridgeStats = {
                    bridgeName: payload.bridge_name,
                    messageCount: payload.bridge_message_count || 0,
                    uptimeSeconds: payload.bridge_uptime_seconds || 0,
                    lastMessageId: payload.message_id || 0
                };
            }

            // 更新状态
            this.messageCount++;
            this.gpsDataReceived = true;
            this.lastMessageTime = new Date();
            
            // 添加到消息历史
            this.addToMessageHistory(this.currentPosition);
            
            // 评估数据质量
            this.assessDataQuality();

            // 更新地图
            this.updateMap();
            
            // 更新界面
            this.updateUI();
            
            // 计算移动距离和速度
            this.calculateMovementStats();
            
            // 获取地址信息
            this.getAddressInfo();
            
            this.showMessage(`GPS数据更新: ${latitude.toFixed(6)}, ${longitude.toFixed(6)}`, 'success');
            
        } catch (error) {
            console.error('处理GPS消息失败:', error);
            this.showMessage('GPS数据处理失败: ' + error.message, 'error');
        }
    }

    /**
     * 添加消息到历史记录
     */
    addToMessageHistory(position) {
        this.messageHistory.push({
            ...position,
            receivedAt: new Date()
        });
        
        if (this.messageHistory.length > this.maxHistorySize) {
            this.messageHistory = this.messageHistory.slice(-this.maxHistorySize);
        }
    }

    /**
     * 评估数据质量
     */
    assessDataQuality() {
        if (this.messageHistory.length < 3) {
            this.dataQuality = 'unknown';
            return;
        }

        const recent = this.messageHistory.slice(-10);
        const now = new Date();
        
        const intervals = [];
        for (let i = 1; i < recent.length; i++) {
            const interval = recent[i].receivedAt - recent[i-1].receivedAt;
            intervals.push(interval);
        }
        
        const avgInterval = intervals.reduce((a, b) => a + b, 0) / intervals.length;
        const maxDeviation = Math.max(...intervals.map(i => Math.abs(i - avgInterval)));
        
        const lastMessageAge = now - this.lastMessageTime;
        
        if (lastMessageAge < 2000 && maxDeviation < 1000) {
            this.dataQuality = 'excellent';
        } else if (lastMessageAge < 5000 && maxDeviation < 3000) {
            this.dataQuality = 'good';
        } else if (lastMessageAge < 10000) {
            this.dataQuality = 'fair';
        } else {
            this.dataQuality = 'poor';
        }
    }

    /**
     * 更新地图显示
     */
    updateMap() {
        if (!this.currentPosition || !this.map) {
            console.warn('地图更新条件不满足');
            return;
        }

        const position = [this.currentPosition.lng, this.currentPosition.lat];

        // 检查并更新标记
        if (!this.marker) {
            try {
                this.marker = new AMap.Marker({
                    position: position,
                    icon: this.createGPSIcon(),
                    title: 'GPS实时位置',
                    anchor: 'center',
                    zIndex: 100
                });
                this.map.add(this.marker);
            } catch (error) {
                console.error('创建GPS标记失败:', error);
                return;
            }
        } else {
            this.marker.setPosition(position);
        }

        // 添加到轨迹
        this.path.push(position);
        
        if (this.path.length > 1000) {
            this.path = this.path.slice(-500);
        }

        // 更新轨迹线
        if (this.polyline && this.showTrail && this.path.length > 1) {
            this.polyline.setPath(this.path);
        }

        // 首次接收数据时自动居中
        if (this.messageCount === 1) {
            this.map.setCenter([this.currentPosition.lng, this.currentPosition.lat]);
            this.map.setZoom(17);
        }
    }

    /**
     * 计算移动统计信息
     */
    calculateMovementStats() {
        if (!this.lastPosition || !this.currentPosition) {
            if (this.currentPosition) {
                this.currentPosition.speed = 0;
            }
            return;
        }

        const distance = this.calculateDistance(
            this.lastPosition.lat, this.lastPosition.lng,
            this.currentPosition.lat, this.currentPosition.lng
        );
        
        this.totalDistance += distance;

        const timeDiff = (this.currentPosition.timestamp - this.lastPosition.timestamp) / 1000;
        
        if (timeDiff > 0 && distance > 0) {
            this.currentPosition.speed = (distance / timeDiff) * 3.6; // m/s to km/h
        } else {
            this.currentPosition.speed = 0;
        }

        // 限制最大速度
        if (this.currentPosition.speed > 200) {
            this.currentPosition.speed = 0;
        }
    }

    /**
     * 计算两点间距离
     */
    calculateDistance(lat1, lng1, lat2, lng2) {
        const R = 6371000;
        const φ1 = lat1 * Math.PI / 180;
        const φ2 = lat2 * Math.PI / 180;
        const Δφ = (lat2 - lat1) * Math.PI / 180;
        const Δλ = (lng2 - lng1) * Math.PI / 180;

        const a = Math.sin(Δφ/2) * Math.sin(Δφ/2) +
                Math.cos(φ1) * Math.cos(φ2) *
                Math.sin(Δλ/2) * Math.sin(Δλ/2);
        const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1-a));

        return R * c;
    }

    /**
     * 更新用户界面
     */
    updateUI() {
        this.updateConnectionStatus();
        this.updateGPSInfo();
        this.updateLocationInfo();
        this.updateBridgeStats();
        this.updateDataQuality();
        this.updateTimestamp();
    }

    /**
     * 更新连接状态显示
     */
    updateConnectionStatus() {
        const wsStatusEl = document.getElementById('wsStatus');
        const gpsStatusEl = document.getElementById('gpsDataStatus');

        if (wsStatusEl) {
            if (this.wsConnected) {
                wsStatusEl.innerHTML = '<span class="status-indicator status-online"></span>已连接';
            } else {
                wsStatusEl.innerHTML = '<span class="status-indicator status-offline"></span>离线';
            }
        }

        if (gpsStatusEl) {
            if (this.gpsDataReceived && this.lastMessageTime) {
                const age = (new Date() - this.lastMessageTime) / 1000;
                if (age < 5) {
                    gpsStatusEl.innerHTML = '<span class="status-indicator status-online"></span>实时';
                } else if (age < 30) {
                    gpsStatusEl.innerHTML = '<span class="status-indicator status-warning"></span>延迟';
                } else {
                    gpsStatusEl.innerHTML = '<span class="status-indicator status-offline"></span>过期';
                }
            } else {
                gpsStatusEl.innerHTML = '<span class="status-indicator status-offline"></span>等待数据';
            }
        }
    }

    /**
     * 更新GPS信息显示
     */
    updateGPSInfo() {
        if (!this.currentPosition) return;

        const coordinatesEl = document.getElementById('coordinates');
        if (coordinatesEl) {
            coordinatesEl.innerHTML = `
                纬度: ${this.currentPosition.lat.toFixed(8)}<br>
                经度: ${this.currentPosition.lng.toFixed(8)}<br>
                消息ID: ${this.currentPosition.messageId}<br>
                源节点: ${this.currentPosition.sourceNode}
            `;
        }

        const speedEl = document.getElementById('speed');
        if (speedEl && this.currentPosition.speed !== undefined) {
            speedEl.textContent = `${this.currentPosition.speed.toFixed(1)} km/h`;
        } else if (speedEl) {
            speedEl.textContent = '0.0 km/h';
        }

        const messageCountEl = document.getElementById('messageCount');
        if (messageCountEl) {
            messageCountEl.textContent = this.messageCount;
        }
    }

    /**
     * 更新位置信息
     */
    updateLocationInfo() {
        const totalDistanceEl = document.getElementById('totalDistance');
        if (totalDistanceEl) {
            totalDistanceEl.textContent = 
                this.totalDistance > 1000 ? 
                `${(this.totalDistance / 1000).toFixed(2)} km` : 
                `${this.totalDistance.toFixed(1)} m`;
        }
    }

    /**
     * 更新桥接器统计信息
     */
    updateBridgeStats() {
        const bridgeMessageCountEl = document.getElementById('bridgeMessageCount');
        const bridgeUptimeEl = document.getElementById('bridgeUptime');
        const bridgeNameEl = document.getElementById('bridgeName');

        if (bridgeMessageCountEl) {
            bridgeMessageCountEl.textContent = this.bridgeStats.messageCount.toLocaleString();
        }

        if (bridgeUptimeEl) {
            const hours = Math.floor(this.bridgeStats.uptimeSeconds / 3600);
            const minutes = Math.floor((this.bridgeStats.uptimeSeconds % 3600) / 60);
            bridgeUptimeEl.textContent = `${hours}h ${minutes}m`;
        }

        if (bridgeNameEl) {
            bridgeNameEl.textContent = this.bridgeStats.bridgeName || '未知';
        }
    }

    /**
     * 更新数据质量指示器
     */
    updateDataQuality() {
        const qualityEl = document.getElementById('dataQuality');
        if (qualityEl) {
            const qualityMap = {
                'excellent': { text: '优秀', class: 'data-quality-excellent' },
                'good': { text: '良好', class: 'data-quality-good' },
                'fair': { text: '一般', class: 'data-quality-fair' },
                'poor': { text: '较差', class: 'data-quality-poor' },
                'unknown': { text: '未知', class: 'data-quality-poor' }
            };

            const quality = qualityMap[this.dataQuality];
            qualityEl.className = `data-quality-indicator ${quality.class}`;
            qualityEl.textContent = quality.text;
        }
    }

    /**
     * 更新时间戳
     */
    updateTimestamp() {
        const timestampEl = document.getElementById('lastUpdate');
        if (timestampEl) {
            timestampEl.textContent = `最后更新: ${new Date().toLocaleString()}`;
        }
    }

    /**
     * 获取地址信息
     */
    getAddressInfo() {
        if (!this.currentPosition || typeof AMap === 'undefined') return;

        AMap.plugin('AMap.Geocoder', () => {
            try {
                const geocoder = new AMap.Geocoder({
                    radius: 1000,
                    extensions: 'all'
                });
                
                const lnglat = [this.currentPosition.lng, this.currentPosition.lat];
                
                geocoder.getAddress(lnglat, (status, result) => {
                    const addressEl = document.getElementById('currentAddress');
                    if (addressEl) {
                        if (status === 'complete' && result.regeocode) {
                            let address = result.regeocode.formattedAddress;
                            if (!address && result.regeocode.addressComponent) {
                                const addr = result.regeocode.addressComponent;
                                address = `${addr.province}${addr.city}${addr.district}${addr.township}${addr.street}${addr.streetNumber}`;
                            }
                            addressEl.textContent = address || '位置信息获取中...';
                        } else {
                            addressEl.textContent = `${this.currentPosition.lat.toFixed(6)}, ${this.currentPosition.lng.toFixed(6)}`;
                        }
                    }
                });
            } catch (error) {
                console.error('地址解析出错:', error);
                const addressEl = document.getElementById('currentAddress');
                if (addressEl) {
                    addressEl.textContent = `坐标: ${this.currentPosition.lat.toFixed(6)}, ${this.currentPosition.lng.toFixed(6)}`;
                }
            }
        });
    }

    /**
     * 设置事件监听器
     */
    setupEventListeners() {
        const trailToggle = document.getElementById('trailToggle');
        if (trailToggle) {
            trailToggle.addEventListener('click', () => {
                this.toggleTrail();
            });
        }

        if (trailToggle) {
            trailToggle.classList.add('active');
        }
    }

    /**
     * 切换轨迹显示
     */
    toggleTrail() {
        this.showTrail = !this.showTrail;
        const trailToggle = document.getElementById('trailToggle');
        
        if (trailToggle) {
            trailToggle.classList.toggle('active', this.showTrail);
        }
        
        if (this.showTrail && this.path.length > 1) {
            this.map.add(this.polyline);
            this.polyline.setPath(this.path);
        } else {
            this.map.remove(this.polyline);
        }
    }

    /**
     * 地图居中到当前位置
     */
    centerMap() {
        if (this.currentPosition && this.map) {
            this.map.setCenter([this.currentPosition.lng, this.currentPosition.lat]);
            this.map.setZoom(17);
            this.showMessage('地图已居中到当前位置', 'info');
        } else {
            this.showMessage('地图居中失败：位置信息或地图不可用', 'warning');
        }
    }

    /**
     * 清除轨迹
     */
    clearTrail() {
        this.path = [];
        
        if (this.polyline) {
            this.polyline.setPath([]);
        }
        
        this.totalDistance = 0;
        
        const totalDistanceEl = document.getElementById('totalDistance');
        if (totalDistanceEl) {
            totalDistanceEl.textContent = '0 m';
        }
        
        this.showMessage('轨迹已清除', 'info');
    }

    /**
     * 显示消息
     */
    showMessage(message, type = 'info') {
        const messageArea = document.getElementById('messageArea');
        if (!messageArea) return;

        const messageDiv = document.createElement('div');
        messageDiv.className = `${type}-message`;
        messageDiv.textContent = message;
        
        messageArea.appendChild(messageDiv);
        
        setTimeout(() => {
            if (messageDiv.parentNode) {
                messageDiv.remove();
            }
        }, 5000);
        
        while (messageArea.children.length > 3) {
            messageArea.removeChild(messageArea.firstChild);
        }
        
        console.log(`[${type.toUpperCase()}] ${message}`);
    }

    /**
     * 启动定期任务
     */
    startPeriodicTasks() {
        setInterval(() => {
            this.updateConnectionStatus();
        }, 1000);
        
        setInterval(() => {
            this.assessDataQuality();
            this.updateDataQuality();
        }, 5000);
    }

    /**
     * 测试GPS数据处理
     */
    testGPSDataProcessing() {
        console.log('=== 测试GPS数据处理 ===');
        
        const testData = {
            "timestamp": "2025-07-24T11:07:21.627317",
            "source_node": "ros_bridge",
            "source_topic": "/gps/fix",
            "data": {
                "latitude": 22.50455341,
                "longitude": 113.8750382736
            },
            "message_id": 4042570,
            "frame_id": "gps_frame",
            "bridge_name": "IMU_bridge",
            "bridge_message_count": 4042570,
            "bridge_uptime_seconds": 153913.900117
        };

        console.log('模拟接收GPS数据:', testData);
        
        this.handleGPSMessage({
            topic: 'ros2/gps/fix',
            payload: testData
        });
        
        console.log('测试完成');
    }
}

// 全局函数
function centerMap() {
    if (window.gpsMonitor) {
        window.gpsMonitor.centerMap();
    }
}

function clearTrail() {
    if (window.gpsMonitor) {
        window.gpsMonitor.clearTrail();
    }
}

function toggleTrail() {
    if (window.gpsMonitor) {
        window.gpsMonitor.toggleTrail();
    }
}

function testGPSData() {
    if (window.gpsMonitor) {
        window.gpsMonitor.testGPSDataProcessing();
    }
}

// 页面加载完成后初始化
document.addEventListener('DOMContentLoaded', () => {
    // 检查高德地图API是否加载
    if (typeof AMap === 'undefined') {
        const messageArea = document.getElementById('messageArea');
        if (messageArea) {
            messageArea.innerHTML = `
                <div class="error-message">
                    高德地图API加载失败，请检查网络连接或API密钥设置。<br>
                    请将HTML文件中的 YOUR_AMAP_KEY 替换为您的高德地图API密钥。
                </div>
            `;
        }
        return;
    }

    // 初始化GPS监控器
    window.gpsMonitor = new GPSMonitor();
});
