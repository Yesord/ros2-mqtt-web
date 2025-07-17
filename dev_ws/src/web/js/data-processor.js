/**
 * 数据处理器类
 */
class DataProcessor {
    constructor() {
        this.messageHistory = [];
        this.statisticsData = {};
        this.lastMessageTime = null;
        this.messageCount = 0;
        this.startTime = Date.now();
        this.lastHeartbeat = null;
        this.frequencyCalculator = new FrequencyCalculator();
    }

    /**
     * 处理Hello World消息
     * @param {Object} data - 消息数据
     */
    processHelloWorldMessage(data) {
        try {
            // 提取消息内容
            const messageContent = data.data || data.message || JSON.stringify(data);
            const messageId = data.message_id || data.bridge_message_count || this.messageCount + 1;
            const timestamp = data.timestamp || new Date().toISOString();

            // 更新消息计数
            this.messageCount = messageId;
            this.lastMessageTime = new Date(timestamp);

            // 计算消息频率
            this.frequencyCalculator.addMessage();

            // 添加到历史记录
            this.addToHistory({
                id: messageId,
                content: messageContent,
                timestamp: timestamp,
                source: data.source_node || 'hello_world_publisher',
                topic: data.source_topic || '/hello_world'
            });

            // 更新UI
            if (window.uiController) {
                window.uiController.updateCurrentMessage(messageContent, messageId);
                window.uiController.addMessageToHistory(messageContent, timestamp, messageId);
            }

            // 记录日志
            this.log(`收到Hello World消息 #${messageId}: ${messageContent}`, 'info');

        } catch (error) {
            this.log(`处理Hello World消息时发生错误: ${error.message}`, 'error');
        }
    }

    /**
     * 处理统计信息
     * @param {Object} data - 统计数据
     */
    processStatistics(data) {
        try {
            this.statisticsData = data;

            // 更新UI统计信息
            if (window.uiController) {
                window.uiController.updateStatistics(data);
            }

            this.log('统计信息已更新', 'info');

        } catch (error) {
            this.log(`处理统计信息时发生错误: ${error.message}`, 'error');
        }
    }

    /**
     * 处理心跳信息
     * @param {Object} data - 心跳数据
     */
    processHeartbeat(data) {
        try {
            this.lastHeartbeat = new Date(data.timestamp || Date.now());

            // 更新运行时间等信息
            if (data.uptime_seconds !== undefined && window.uiController) {
                window.uiController.updateStatistics({
                    uptime_seconds: data.uptime_seconds,
                    total_published_messages: data.total_published_messages,
                    status: data.status || 'alive'
                });
            }

            this.log('收到心跳信号', 'info');

        } catch (error) {
            this.log(`处理心跳信息时发生错误: ${error.message}`, 'error');
        }
    }

    /**
     * 处理历史消息
     * @param {Object} data - 历史数据
     */
    processHistory(data) {
        try {
            const history = data.message_history || [];
            
            // 清空当前历史显示
            if (window.uiController) {
                window.uiController.clearMessageHistory();
            }

            // 添加历史消息到UI
            history.forEach((item, index) => {
                if (window.uiController) {
                    window.uiController.addMessageToHistory(
                        item.content || item.message || JSON.stringify(item),
                        item.timestamp || new Date().toISOString(),
                        item.id || index + 1
                    );
                }
            });

            this.log(`已加载 ${history.length} 条历史消息`, 'success');

        } catch (error) {
            this.log(`处理历史消息时发生错误: ${error.message}`, 'error');
        }
    }

    /**
     * 处理桥接状态
     * @param {Object} data - 状态数据
     */
    processBridgeStatus(data) {
        try {
            const status = data.status || 'unknown';
            
            // 更新状态显示
            if (window.uiController) {
                window.uiController.updateStatistics({
                    status: status,
                    uptime_seconds: data.uptime_seconds,
                    total_published_messages: data.total_published_messages,
                    mqtt_connected: data.mqtt_connected
                });
            }

            this.log(`桥接节点状态: ${status}`, 'info');

        } catch (error) {
            this.log(`处理桥接状态时发生错误: ${error.message}`, 'error');
        }
    }

    /**
     * 处理重置确认
     * @param {Object} data - 重置确认数据
     */
    processResetConfirm(data) {
        try {
            // 重置本地统计
            this.messageCount = 0;
            this.messageHistory = [];
            this.startTime = Date.now();
            this.frequencyCalculator.reset();

            // 更新UI
            if (window.uiController) {
                window.uiController.updateStatistics({
                    total_published_messages: 0,
                    uptime_seconds: 0,
                    status: 'reset'
                });
                window.uiController.updateCurrentMessage('等待消息...', 0);
            }

            this.log('统计信息已重置', 'success');

        } catch (error) {
            this.log(`处理重置确认时发生错误: ${error.message}`, 'error');
        }
    }

    /**
     * 添加消息到历史记录
     * @param {Object} message - 消息对象
     */
    addToHistory(message) {
        this.messageHistory.unshift(message);
        
        // 限制历史记录数量
        const maxHistory = 100;
        if (this.messageHistory.length > maxHistory) {
            this.messageHistory = this.messageHistory.slice(0, maxHistory);
        }
    }

    /**
     * 获取消息历史
     * @param {number} limit - 限制数量
     * @returns {Array} 历史消息数组
     */
    getMessageHistory(limit = 20) {
        return this.messageHistory.slice(0, limit);
    }

    /**
     * 获取统计数据
     * @returns {Object} 统计数据对象
     */
    getStatistics() {
        const now = Date.now();
        const uptime = (now - this.startTime) / 1000;
        const frequency = this.frequencyCalculator.getFrequency();

        return {
            messageCount: this.messageCount,
            uptime: uptime,
            frequency: frequency,
            lastMessageTime: this.lastMessageTime,
            lastHeartbeat: this.lastHeartbeat,
            historySize: this.messageHistory.length,
            ...this.statisticsData
        };
    }

    /**
     * 清空历史记录
     */
    clearHistory() {
        this.messageHistory = [];
        this.log('历史记录已清空', 'info');
    }

    /**
     * 重置统计数据
     */
    resetStatistics() {
        this.messageCount = 0;
        this.messageHistory = [];
        this.startTime = Date.now();
        this.lastMessageTime = null;
        this.lastHeartbeat = null;
        this.frequencyCalculator.reset();
        this.statisticsData = {};
        this.log('统计数据已重置', 'info');
    }

    /**
     * 导出数据
     * @returns {Object} 导出的数据对象
     */
    exportData() {
        return {
            timestamp: new Date().toISOString(),
            statistics: this.getStatistics(),
            messageHistory: this.messageHistory,
            frequencyData: this.frequencyCalculator.getHistoryData()
        };
    }

    /**
     * 记录日志
     * @param {string} message - 日志消息
     * @param {string} level - 日志级别
     */
    log(message, level = 'info') {
        if (window.uiController) {
            window.uiController.addLogEntry(`[DataProcessor] ${message}`, level);
        }
    }
}

/**
 * 频率计算器类
 */
class FrequencyCalculator {
    constructor(windowSize = 60) {
        this.windowSize = windowSize; // 时间窗口大小（秒）
        this.timestamps = [];
        this.historyData = [];
    }

    /**
     * 添加消息时间戳
     */
    addMessage() {
        const now = Date.now();
        this.timestamps.push(now);

        // 移除超出时间窗口的数据
        const cutoff = now - (this.windowSize * 1000);
        this.timestamps = this.timestamps.filter(ts => ts > cutoff);

        // 记录历史数据用于图表显示
        this.historyData.push({
            timestamp: now,
            frequency: this.getFrequency()
        });

        // 限制历史数据大小
        if (this.historyData.length > 1000) {
            this.historyData = this.historyData.slice(-500);
        }
    }

    /**
     * 获取当前频率
     * @returns {number} 消息频率 (msg/s)
     */
    getFrequency() {
        if (this.timestamps.length < 2) {
            return 0;
        }

        const now = Date.now();
        const windowStart = now - (this.windowSize * 1000);
        const messagesInWindow = this.timestamps.filter(ts => ts > windowStart).length;

        return messagesInWindow / this.windowSize;
    }

    /**
     * 获取历史数据
     * @returns {Array} 历史频率数据
     */
    getHistoryData() {
        return this.historyData;
    }

    /**
     * 重置计算器
     */
    reset() {
        this.timestamps = [];
        this.historyData = [];
    }
}

/**
 * 数据验证器
 */
class DataValidator {
    /**
     * 验证Hello World消息
     * @param {Object} data - 消息数据
     * @returns {boolean} 是否有效
     */
    static validateHelloWorldMessage(data) {
        return data && (data.data || data.message || typeof data === 'string');
    }

    /**
     * 验证统计数据
     * @param {Object} data - 统计数据
     * @returns {boolean} 是否有效
     */
    static validateStatistics(data) {
        return data && typeof data === 'object';
    }

    /**
     * 验证心跳数据
     * @param {Object} data - 心跳数据
     * @returns {boolean} 是否有效
     */
    static validateHeartbeat(data) {
        return data && (data.status || data.timestamp);
    }

    /**
     * 验证历史数据
     * @param {Object} data - 历史数据
     * @returns {boolean} 是否有效
     */
    static validateHistory(data) {
        return data && Array.isArray(data.message_history);
    }
}

// 创建全局数据处理器实例
window.dataProcessor = new DataProcessor();
window.DataValidator = DataValidator;
