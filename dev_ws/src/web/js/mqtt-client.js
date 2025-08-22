/**
 * MQTT客户端管理类
 */
class MQTTClient {
    constructor() {
        this.client = null;
        this.isConnected = false;
        this.config = {
            host: 'localhost',
            port: 9001,
            username: '',
            password: '',
            topicPrefix: 'ros2'
        };
        this.subscriptions = new Map();
        this.messageHandlers = new Map();
        this.reconnectAttempts = 0;
        this.maxReconnectAttempts = 5;
        this.reconnectDelay = 3000;
    }

    /**
     * 更新配置
     * @param {Object} config - 新的配置对象
     */
    updateConfig(config) {
        this.config = { ...this.config, ...config };
        this.log('配置已更新', 'info');
    }

    /**
     * 连接到MQTT服务器
     */
    connect() {
        if (this.isConnected) {
            this.log('已经连接到MQTT服务器', 'warning');
            return;
        }

        this.log('正在连接到MQTT服务器...', 'info');
        
        const options = {
            keepalive: 60,
            clientId: 'web_dashboard_' + Math.random().toString(16).substr(2, 8),
            protocolId: 'MQTT',
            protocolVersion: 4,
            clean: true,
            reconnectPeriod: 1000,
            connectTimeout: 30 * 1000,
            will: {
                topic: `${this.config.topicPrefix}/dashboard/status`,
                payload: JSON.stringify({
                    status: 'offline',
                    timestamp: new Date().toISOString()
                }),
                qos: 0,
                retain: false
            }
        };

        // 添加认证信息（如果有）
        if (this.config.username && this.config.password) {
            options.username = this.config.username;
            options.password = this.config.password;
        }

        try {
            // 使用WebSocket连接
            const brokerUrl = `ws://${this.config.host}:${this.config.port}/mqtt`;
            this.client = mqtt.connect(brokerUrl, options);

            // 设置事件监听器
            this.setupEventHandlers();

        } catch (error) {
            this.log(`连接失败: ${error.message}`, 'error');
            this.onConnectionFailed();
        }
    }

    /**
     * 设置事件处理器
     */
    setupEventHandlers() {
        // 连接成功
        this.client.on('connect', () => {
            this.isConnected = true;
            this.reconnectAttempts = 0;
            this.log('✓ MQTT连接成功', 'success');
            this.onConnected();
            this.subscribeToDefaultTopics();
        });

        // 连接失败
        this.client.on('error', (error) => {
            this.log(`MQTT错误: ${error.message}`, 'error');
            this.onConnectionFailed();
        });

        // 断开连接
        this.client.on('close', () => {
            this.isConnected = false;
            this.log('MQTT连接已断开', 'warning');
            this.onDisconnected();
        });

        // 重连
        this.client.on('reconnect', () => {
            this.reconnectAttempts++;
            this.log(`正在重连... (${this.reconnectAttempts}/${this.maxReconnectAttempts})`, 'info');
            
            if (this.reconnectAttempts >= this.maxReconnectAttempts) {
                this.log('达到最大重连次数，停止重连', 'error');
                this.client.end();
            }
        });

        // 离线
        this.client.on('offline', () => {
            this.isConnected = false;
            this.log('MQTT客户端离线', 'warning');
            this.onDisconnected();
        });

        // 接收消息
        this.client.on('message', (topic, message, packet) => {
            this.handleMessage(topic, message, packet);
        });
    }

    /**
     * 订阅默认主题
     */
    subscribeToDefaultTopics() {
        const topics = [
            `${this.config.topicPrefix}/hello_world/data`,
            `${this.config.topicPrefix}/hello_world/statistics`,
            `${this.config.topicPrefix}/hello_world/heartbeat`,
            `${this.config.topicPrefix}/hello_world/history`,
            `${this.config.topicPrefix}/hello_world/bridge_status`,
            `${this.config.topicPrefix}/hello_world/reset_confirm`
        ];

        topics.forEach(topic => {
            this.subscribe(topic);
        });
    }

    /**
     * 订阅主题
     * @param {string} topic - 主题名称
     * @param {number} qos - QoS级别
     */
    subscribe(topic, qos = 0) {
        if (!this.isConnected) {
            this.log('未连接到MQTT服务器，无法订阅主题', 'error');
            return;
        }

        this.client.subscribe(topic, { qos }, (error) => {
            if (error) {
                this.log(`订阅主题失败 ${topic}: ${error.message}`, 'error');
            } else {
                this.subscriptions.set(topic, qos);
                this.log(`✓ 已订阅主题: ${topic}`, 'success');
            }
        });
    }

    /**
     * 取消订阅主题
     * @param {string} topic - 主题名称
     */
    unsubscribe(topic) {
        if (!this.isConnected) {
            this.log('未连接到MQTT服务器，无法取消订阅', 'error');
            return;
        }

        this.client.unsubscribe(topic, (error) => {
            if (error) {
                this.log(`取消订阅失败 ${topic}: ${error.message}`, 'error');
            } else {
                this.subscriptions.delete(topic);
                this.log(`✓ 已取消订阅: ${topic}`, 'success');
            }
        });
    }

    /**
     * 发布消息
     * @param {string} topic - 主题名称
     * @param {string|Object} message - 消息内容
     * @param {number} qos - QoS级别
     */
    publish(topic, message, qos = 0) {
        if (!this.isConnected) {
            this.log('未连接到MQTT服务器，无法发布消息', 'error');
            return;
        }

        const payload = typeof message === 'string' ? message : JSON.stringify(message);
        
        this.client.publish(topic, payload, { qos }, (error) => {
            if (error) {
                this.log(`发布消息失败 ${topic}: ${error.message}`, 'error');
            } else {
                this.log(`✓ 消息已发布到: ${topic}`, 'success');
            }
        });
    }

    /**
     * 处理接收到的消息
     * @param {string} topic - 主题名称
     * @param {Buffer} message - 消息内容
     * @param {Object} packet - 消息包
     */
    handleMessage(topic, message, packet) {
        try {
            const messageStr = message.toString();
            let messageData;
            
            // 尝试解析JSON
            try {
                messageData = JSON.parse(messageStr);
            } catch (e) {
                messageData = messageStr;
            }

            this.log(`收到消息 [${topic}]: ${messageStr.substring(0, 100)}${messageStr.length > 100 ? '...' : ''}`, 'info');

            // 调用注册的消息处理器
            if (this.messageHandlers.has(topic)) {
                this.messageHandlers.get(topic)(messageData, topic);
            }

            // 根据主题类型处理消息
            this.routeMessage(topic, messageData);

        } catch (error) {
            this.log(`处理消息时发生错误: ${error.message}`, 'error');
        }
    }

    /**
     * 路由消息到相应的处理函数
     * @param {string} topic - 主题名称
     * @param {*} messageData - 消息数据
     */
    routeMessage(topic, messageData) {
        const topicParts = topic.split('/');
        const messageType = topicParts[topicParts.length - 1];

        switch (messageType) {
            case 'data':
                this.handleHelloWorldData(messageData);
                break;
            case 'statistics':
                this.handleStatistics(messageData);
                break;
            case 'heartbeat':
                this.handleHeartbeat(messageData);
                break;
            case 'history':
                this.handleHistory(messageData);
                break;
            case 'bridge_status':
                this.handleBridgeStatus(messageData);
                break;
            case 'reset_confirm':
                this.handleResetConfirm(messageData);
                break;
            default:
                this.log(`未知的消息类型: ${messageType}`, 'warning');
        }
    }

    /**
     * 处理Hello World数据
     * @param {Object} data - 消息数据
     */
    handleHelloWorldData(data) {
        if (window.dataProcessor) {
            window.dataProcessor.processHelloWorldMessage(data);
        }
    }

    /**
     * 处理统计信息
     * @param {Object} data - 统计数据
     */
    handleStatistics(data) {
        if (window.dataProcessor) {
            window.dataProcessor.processStatistics(data);
        }
    }

    /**
     * 处理心跳信息
     * @param {Object} data - 心跳数据
     */
    handleHeartbeat(data) {
        if (window.dataProcessor) {
            window.dataProcessor.processHeartbeat(data);
        }
    }

    /**
     * 处理历史消息
     * @param {Object} data - 历史数据
     */
    handleHistory(data) {
        if (window.dataProcessor) {
            window.dataProcessor.processHistory(data);
        }
    }

    /**
     * 处理桥接状态
     * @param {Object} data - 状态数据
     */
    handleBridgeStatus(data) {
        if (window.dataProcessor) {
            window.dataProcessor.processBridgeStatus(data);
        }
    }

    /**
     * 处理重置确认
     * @param {Object} data - 重置确认数据
     */
    handleResetConfirm(data) {
        this.log('统计信息已重置', 'success');
        if (window.dataProcessor) {
            window.dataProcessor.processResetConfirm(data);
        }
    }

    /**
     * 注册消息处理器
     * @param {string} topic - 主题名称
     * @param {Function} handler - 处理函数
     */
    registerMessageHandler(topic, handler) {
        this.messageHandlers.set(topic, handler);
    }

    /**
     * 发送控制命令
     * @param {string} command - 命令类型
     * @param {Object} params - 命令参数
     */
    sendControlCommand(command, params = {}) {
        const controlTopic = `${this.config.topicPrefix}/hello_world/control`;
        const message = {
            command: command,
            timestamp: new Date().toISOString(),
            ...params
        };

        this.publish(controlTopic, message);
    }

    /**
     * 断开连接
     */
    disconnect() {
        if (this.client && this.isConnected) {
            this.log('正在断开MQTT连接...', 'info');
            this.client.end();
            this.isConnected = false;
        }
    }

    /**
     * 连接成功回调
     */
    onConnected() {
        if (window.uiController) {
            window.uiController.updateConnectionStatus(true);
        }
        
        // 发布上线消息
        this.publish(`${this.config.topicPrefix}/dashboard/status`, {
            status: 'online',
            timestamp: new Date().toISOString(),
            userAgent: navigator.userAgent
        });
    }

    /**
     * 连接失败回调
     */
    onConnectionFailed() {
        if (window.uiController) {
            window.uiController.updateConnectionStatus(false);
        }
    }

    /**
     * 断开连接回调
     */
    onDisconnected() {
        if (window.uiController) {
            window.uiController.updateConnectionStatus(false);
        }
    }

    /**
     * 记录日志
     * @param {string} message - 日志消息
     * @param {string} level - 日志级别
     */
    log(message, level = 'info') {
        const timestamp = new Date().toLocaleTimeString();
        console.log(`[${timestamp}] [${level.toUpperCase()}] ${message}`);
        
        if (window.uiController) {
            window.uiController.addLogEntry(message, level);
        }
    }

    /**
     * 获取连接状态
     * @returns {boolean} 是否已连接
     */
    isConnectedToMQTT() {
        return this.isConnected;
    }

    /**
     * 获取订阅列表
     * @returns {Map} 订阅列表
     */
    getSubscriptions() {
        return this.subscriptions;
    }
}

// 创建全局MQTT客户端实例
window.mqttClient = new MQTTClient();
