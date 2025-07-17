/**
 * UI控制器类
 */
class UIController {
    constructor() {
        this.elements = {};
        this.autoScroll = true;
        this.initializeElements();
        this.setupEventListeners();
    }

    /**
     * 初始化DOM元素引用
     */
    initializeElements() {
        this.elements = {
            // 连接状态
            mqttStatus: document.getElementById('mqtt-status'),
            
            // 配置表单
            mqttHost: document.getElementById('mqtt-host'),
            mqttPort: document.getElementById('mqtt-port'),
            mqttTopicPrefix: document.getElementById('mqtt-topic-prefix'),
            mqttUsername: document.getElementById('mqtt-username'),
            mqttPassword: document.getElementById('mqtt-password'),
            connectBtn: document.getElementById('connect-btn'),
            disconnectBtn: document.getElementById('disconnect-btn'),
            
            // 消息显示
            currentMessageContent: document.getElementById('current-message-content'),
            messageCount: document.getElementById('message-count'),
            lastUpdate: document.getElementById('last-update'),
            messageHistoryList: document.getElementById('message-history-list'),
            clearHistoryBtn: document.getElementById('clear-history-btn'),
            
            // 统计信息
            bridgeStatus: document.getElementById('bridge-status'),
            bridgeUptime: document.getElementById('bridge-uptime'),
            totalPublished: document.getElementById('total-published'),
            messageFrequency: document.getElementById('message-frequency'),
            refreshStatsBtn: document.getElementById('refresh-stats-btn'),
            resetStatsBtn: document.getElementById('reset-stats-btn'),
            
            // 控制按钮
            getHistoryBtn: document.getElementById('get-history-btn'),
            heartbeatBtn: document.getElementById('heartbeat-btn'),
            exportDataBtn: document.getElementById('export-data-btn'),
            
            // 日志
            logContent: document.getElementById('log-content'),
            clearLogBtn: document.getElementById('clear-log-btn'),
            autoScrollBtn: document.getElementById('auto-scroll-btn')
        };
    }

    /**
     * 设置事件监听器
     */
    setupEventListeners() {
        // 连接按钮
        this.elements.connectBtn.addEventListener('click', () => {
            this.connectToMQTT();
        });

        // 断开按钮
        this.elements.disconnectBtn.addEventListener('click', () => {
            this.disconnectFromMQTT();
        });

        // 清空历史按钮
        this.elements.clearHistoryBtn.addEventListener('click', () => {
            this.clearMessageHistory();
        });

        // 刷新统计按钮
        this.elements.refreshStatsBtn.addEventListener('click', () => {
            this.refreshStatistics();
        });

        // 重置统计按钮
        this.elements.resetStatsBtn.addEventListener('click', () => {
            this.resetStatistics();
        });

        // 获取历史按钮
        this.elements.getHistoryBtn.addEventListener('click', () => {
            this.getMessageHistory();
        });

        // 心跳按钮
        this.elements.heartbeatBtn.addEventListener('click', () => {
            this.sendHeartbeat();
        });

        // 导出数据按钮
        this.elements.exportDataBtn.addEventListener('click', () => {
            this.exportData();
        });

        // 清空日志按钮
        this.elements.clearLogBtn.addEventListener('click', () => {
            this.clearLog();
        });

        // 自动滚动按钮
        this.elements.autoScrollBtn.addEventListener('click', () => {
            this.toggleAutoScroll();
        });

        // 回车键连接
        [this.elements.mqttHost, this.elements.mqttPort].forEach(element => {
            element.addEventListener('keypress', (e) => {
                if (e.key === 'Enter') {
                    this.connectToMQTT();
                }
            });
        });
    }

    /**
     * 连接到MQTT服务器
     */
    connectToMQTT() {
        const config = {
            host: this.elements.mqttHost.value.trim() || 'localhost',
            port: parseInt(this.elements.mqttPort.value) || 8083,
            username: this.elements.mqttUsername.value.trim(),
            password: this.elements.mqttPassword.value.trim(),
            topicPrefix: this.elements.mqttTopicPrefix.value.trim() || 'ros2'
        };

        // 验证配置
        if (!config.host) {
            this.showError('请输入有效的MQTT服务器地址');
            return;
        }

        if (config.port < 1 || config.port > 65535) {
            this.showError('请输入有效的端口号 (1-65535)');
            return;
        }

        // 更新按钮状态
        this.elements.connectBtn.disabled = true;
        this.elements.connectBtn.innerHTML = '<i class="fas fa-spinner fa-spin"></i> 连接中...';

        // 更新MQTT客户端配置并连接
        window.mqttClient.updateConfig(config);
        window.mqttClient.connect();

        // 3秒后重置按钮状态（防止卡死）
        setTimeout(() => {
            if (!window.mqttClient.isConnectedToMQTT()) {
                this.elements.connectBtn.disabled = false;
                this.elements.connectBtn.innerHTML = '<i class="fas fa-plug"></i> 连接';
            }
        }, 3000);
    }

    /**
     * 断开MQTT连接
     */
    disconnectFromMQTT() {
        window.mqttClient.disconnect();
    }

    /**
     * 更新连接状态
     * @param {boolean} connected - 是否已连接
     */
    updateConnectionStatus(connected) {
        if (connected) {
            this.elements.mqttStatus.innerHTML = '<i class="fas fa-circle"></i> MQTT 已连接';
            this.elements.mqttStatus.className = 'status-indicator connected';
            this.elements.connectBtn.disabled = true;
            this.elements.connectBtn.innerHTML = '<i class="fas fa-check"></i> 已连接';
            this.elements.disconnectBtn.disabled = false;
        } else {
            this.elements.mqttStatus.innerHTML = '<i class="fas fa-circle"></i> MQTT 未连接';
            this.elements.mqttStatus.className = 'status-indicator disconnected';
            this.elements.connectBtn.disabled = false;
            this.elements.connectBtn.innerHTML = '<i class="fas fa-plug"></i> 连接';
            this.elements.disconnectBtn.disabled = true;
        }
    }

    /**
     * 更新当前消息
     * @param {string} message - 消息内容
     * @param {number} count - 消息计数
     */
    updateCurrentMessage(message, count = 0) {
        this.elements.currentMessageContent.textContent = message;
        this.elements.messageCount.textContent = `消息数: ${count}`;
        this.elements.lastUpdate.textContent = `最后更新: ${new Date().toLocaleString()}`;
        
        // 添加脉冲效果
        this.elements.currentMessageContent.classList.add('pulse');
        setTimeout(() => {
            this.elements.currentMessageContent.classList.remove('pulse');
        }, 2000);
    }

    /**
     * 添加消息到历史记录
     * @param {string} message - 消息内容
     * @param {string} timestamp - 时间戳
     * @param {number} messageId - 消息ID
     */
    addMessageToHistory(message, timestamp, messageId) {
        const historyItem = document.createElement('div');
        historyItem.className = 'history-item';
        historyItem.innerHTML = `
            <div class="history-item-header">
                <span>消息 #${messageId}</span>
                <span>${new Date(timestamp).toLocaleString()}</span>
            </div>
            <div class="history-item-content">${message}</div>
        `;

        this.elements.messageHistoryList.prepend(historyItem);

        // 限制历史记录数量
        const maxHistory = 20;
        while (this.elements.messageHistoryList.children.length > maxHistory) {
            this.elements.messageHistoryList.removeChild(this.elements.messageHistoryList.lastChild);
        }

        // 自动滚动到顶部
        this.elements.messageHistoryList.scrollTop = 0;
    }

    /**
     * 清空消息历史
     */
    clearMessageHistory() {
        this.elements.messageHistoryList.innerHTML = '';
        this.addLogEntry('消息历史已清空', 'info');
    }

    /**
     * 更新统计信息
     * @param {Object} stats - 统计数据
     */
    updateStatistics(stats) {
        if (stats.status) {
            this.elements.bridgeStatus.textContent = stats.status;
        }

        if (stats.uptime_seconds !== undefined) {
            this.elements.bridgeUptime.textContent = this.formatUptime(stats.uptime_seconds);
        }

        if (stats.total_published_messages !== undefined) {
            this.elements.totalPublished.textContent = stats.total_published_messages;
        }

        if (stats.hello_world_stats && stats.hello_world_stats.message_frequency) {
            this.elements.messageFrequency.textContent = `${stats.hello_world_stats.message_frequency.toFixed(2)} msg/s`;
        }
    }

    /**
     * 格式化运行时间
     * @param {number} seconds - 秒数
     * @returns {string} 格式化后的时间
     */
    formatUptime(seconds) {
        const hours = Math.floor(seconds / 3600);
        const minutes = Math.floor((seconds % 3600) / 60);
        const secs = Math.floor(seconds % 60);
        
        if (hours > 0) {
            return `${hours}h ${minutes}m ${secs}s`;
        } else if (minutes > 0) {
            return `${minutes}m ${secs}s`;
        } else {
            return `${secs}s`;
        }
    }

    /**
     * 刷新统计信息
     */
    refreshStatistics() {
        if (window.mqttClient.isConnectedToMQTT()) {
            window.mqttClient.sendControlCommand('get_stats');
            this.addLogEntry('已请求刷新统计信息', 'info');
        } else {
            this.showError('请先连接到MQTT服务器');
        }
    }

    /**
     * 重置统计信息
     */
    resetStatistics() {
        if (window.mqttClient.isConnectedToMQTT()) {
            if (confirm('确定要重置统计信息吗？')) {
                window.mqttClient.sendControlCommand('reset_stats');
                this.addLogEntry('已请求重置统计信息', 'info');
            }
        } else {
            this.showError('请先连接到MQTT服务器');
        }
    }

    /**
     * 获取消息历史
     */
    getMessageHistory() {
        if (window.mqttClient.isConnectedToMQTT()) {
            window.mqttClient.sendControlCommand('get_history');
            this.addLogEntry('已请求获取消息历史', 'info');
        } else {
            this.showError('请先连接到MQTT服务器');
        }
    }

    /**
     * 发送心跳
     */
    sendHeartbeat() {
        if (window.mqttClient.isConnectedToMQTT()) {
            const heartbeatTopic = `${window.mqttClient.config.topicPrefix}/dashboard/heartbeat`;
            window.mqttClient.publish(heartbeatTopic, {
                status: 'ping',
                timestamp: new Date().toISOString()
            });
            this.addLogEntry('心跳信号已发送', 'success');
        } else {
            this.showError('请先连接到MQTT服务器');
        }
    }

    /**
     * 导出数据
     */
    exportData() {
        const data = {
            timestamp: new Date().toISOString(),
            currentMessage: this.elements.currentMessageContent.textContent,
            messageCount: this.elements.messageCount.textContent,
            bridgeStatus: this.elements.bridgeStatus.textContent,
            bridgeUptime: this.elements.bridgeUptime.textContent,
            totalPublished: this.elements.totalPublished.textContent,
            messageFrequency: this.elements.messageFrequency.textContent,
            messageHistory: Array.from(this.elements.messageHistoryList.children).map(item => ({
                header: item.querySelector('.history-item-header').textContent,
                content: item.querySelector('.history-item-content').textContent
            })),
            logs: Array.from(this.elements.logContent.children).map(entry => ({
                timestamp: entry.querySelector('.timestamp').textContent,
                level: entry.querySelector('.level').textContent,
                message: entry.querySelector('.message').textContent
            }))
        };

        const jsonStr = JSON.stringify(data, null, 2);
        const blob = new Blob([jsonStr], { type: 'application/json' });
        const url = URL.createObjectURL(blob);
        
        const a = document.createElement('a');
        a.href = url;
        a.download = `ros2-mqtt-data-${new Date().toISOString().split('T')[0]}.json`;
        document.body.appendChild(a);
        a.click();
        document.body.removeChild(a);
        URL.revokeObjectURL(url);

        this.addLogEntry('数据已导出', 'success');
    }

    /**
     * 添加日志条目
     * @param {string} message - 日志消息
     * @param {string} level - 日志级别
     */
    addLogEntry(message, level = 'info') {
        const logEntry = document.createElement('div');
        logEntry.className = `log-entry ${level}`;
        logEntry.innerHTML = `
            <span class="timestamp">[${new Date().toLocaleTimeString()}]</span>
            <span class="level">${level.toUpperCase()}</span>
            <span class="message">${message}</span>
        `;

        this.elements.logContent.appendChild(logEntry);

        // 限制日志数量
        const maxLogs = 100;
        while (this.elements.logContent.children.length > maxLogs) {
            this.elements.logContent.removeChild(this.elements.logContent.firstChild);
        }

        // 自动滚动到底部
        if (this.autoScroll) {
            this.elements.logContent.scrollTop = this.elements.logContent.scrollHeight;
        }
    }

    /**
     * 清空日志
     */
    clearLog() {
        this.elements.logContent.innerHTML = '';
        this.addLogEntry('日志已清空', 'info');
    }

    /**
     * 切换自动滚动
     */
    toggleAutoScroll() {
        this.autoScroll = !this.autoScroll;
        
        if (this.autoScroll) {
            this.elements.autoScrollBtn.classList.add('active');
            this.elements.autoScrollBtn.innerHTML = '<i class="fas fa-check"></i> 自动滚动';
            this.elements.logContent.scrollTop = this.elements.logContent.scrollHeight;
        } else {
            this.elements.autoScrollBtn.classList.remove('active');
            this.elements.autoScrollBtn.innerHTML = '<i class="fas fa-times"></i> 自动滚动';
        }
    }

    /**
     * 显示错误信息
     * @param {string} message - 错误消息
     */
    showError(message) {
        this.addLogEntry(message, 'error');
        
        // 创建临时错误提示
        const errorDiv = document.createElement('div');
        errorDiv.className = 'error-message';
        errorDiv.textContent = message;
        
        // 找到合适的位置插入错误消息
        const configPanel = document.querySelector('.config-panel');
        configPanel.appendChild(errorDiv);
        
        // 3秒后自动移除
        setTimeout(() => {
            if (errorDiv.parentNode) {
                errorDiv.parentNode.removeChild(errorDiv);
            }
        }, 3000);
    }

    /**
     * 显示成功信息
     * @param {string} message - 成功消息
     */
    showSuccess(message) {
        this.addLogEntry(message, 'success');
        
        // 创建临时成功提示
        const successDiv = document.createElement('div');
        successDiv.className = 'success-message';
        successDiv.textContent = message;
        
        // 找到合适的位置插入成功消息
        const configPanel = document.querySelector('.config-panel');
        configPanel.appendChild(successDiv);
        
        // 3秒后自动移除
        setTimeout(() => {
            if (successDiv.parentNode) {
                successDiv.parentNode.removeChild(successDiv);
            }
        }, 3000);
    }

    /**
     * 设置加载状态
     * @param {boolean} loading - 是否加载中
     */
    setLoading(loading) {
        const dashboard = document.querySelector('.dashboard');
        if (loading) {
            dashboard.classList.add('loading');
        } else {
            dashboard.classList.remove('loading');
        }
    }
}

// 创建全局UI控制器实例
window.uiController = new UIController();
