/**
 * 主应用程序
 */
class App {
    constructor() {
        this.initialized = false;
        this.config = this.loadConfig();
        this.init();
    }

    /**
     * 初始化应用程序
     */
    init() {
        if (this.initialized) return;

        this.log('正在初始化ROS2 MQTT监控面板...', 'info');

        // 等待DOM加载完成
        if (document.readyState === 'loading') {
            document.addEventListener('DOMContentLoaded', () => {
                this.initializeApp();
            });
        } else {
            this.initializeApp();
        }
    }

    /**
     * 初始化应用程序组件
     */
    initializeApp() {
        try {
            // 检查必要的组件是否已加载
            this.checkDependencies();

            // 初始化配置
            this.initializeConfig();

            // 设置全局错误处理
            this.setupErrorHandling();

            // 设置页面生命周期事件
            this.setupPageLifecycle();

            // 设置定时任务
            this.setupTimers();

            // 设置键盘快捷键
            this.setupKeyboardShortcuts();

            // 标记为已初始化
            this.initialized = true;

            this.log('✓ 应用程序初始化完成', 'success');

            // 自动连接（如果配置了自动连接）
            if (this.config.autoConnect) {
                setTimeout(() => {
                    this.autoConnect();
                }, 1000);
            }

        } catch (error) {
            this.log(`应用程序初始化失败: ${error.message}`, 'error');
            console.error('App initialization failed:', error);
        }
    }

    /**
     * 检查依赖项
     */
    checkDependencies() {
        const required = ['mqttClient', 'uiController', 'dataProcessor'];
        const missing = required.filter(dep => !window[dep]);

        if (missing.length > 0) {
            throw new Error(`缺少必要的依赖项: ${missing.join(', ')}`);
        }

        // 检查MQTT.js库
        if (typeof mqtt === 'undefined') {
            throw new Error('MQTT.js库未加载');
        }

        this.log('✓ 所有依赖项检查通过', 'success');
    }

    /**
     * 初始化配置
     */
    initializeConfig() {
        // 从localStorage恢复配置
        const savedConfig = localStorage.getItem('mqtt-dashboard-config');
        if (savedConfig) {
            try {
                this.config = { ...this.config, ...JSON.parse(savedConfig) };
                this.log('✓ 配置已从本地存储恢复', 'info');
            } catch (error) {
                this.log('恢复配置失败，使用默认配置', 'warning');
            }
        }

        // 应用配置到UI
        this.applyConfigToUI();
    }

    /**
     * 应用配置到UI
     */
    applyConfigToUI() {
        if (window.uiController && window.uiController.elements) {
            const elements = window.uiController.elements;
            
            if (elements.mqttHost) elements.mqttHost.value = this.config.host || 'localhost';
            if (elements.mqttPort) elements.mqttPort.value = this.config.port || 8083;
            if (elements.mqttTopicPrefix) elements.mqttTopicPrefix.value = this.config.topicPrefix || 'ros2';
            if (elements.mqttUsername) elements.mqttUsername.value = this.config.username || '';
            if (elements.mqttPassword) elements.mqttPassword.value = this.config.password || '';
        }
    }

    /**
     * 设置错误处理
     */
    setupErrorHandling() {
        // 全局错误处理
        window.addEventListener('error', (event) => {
            this.log(`全局错误: ${event.error.message}`, 'error');
            console.error('Global error:', event.error);
        });

        // Promise错误处理
        window.addEventListener('unhandledrejection', (event) => {
            this.log(`未处理的Promise错误: ${event.reason}`, 'error');
            console.error('Unhandled promise rejection:', event.reason);
        });

        // MQTT错误处理
        if (window.mqttClient) {
            window.mqttClient.on = window.mqttClient.on || function() {};
        }
    }

    /**
     * 设置页面生命周期事件
     */
    setupPageLifecycle() {
        // 页面可见性变化
        document.addEventListener('visibilitychange', () => {
            if (document.hidden) {
                this.log('页面已隐藏', 'info');
                this.onPageHidden();
            } else {
                this.log('页面已显示', 'info');
                this.onPageVisible();
            }
        });

        // 页面关闭前
        window.addEventListener('beforeunload', (event) => {
            this.onPageUnload();
        });

        // 页面大小改变
        window.addEventListener('resize', () => {
            this.onWindowResize();
        });
    }

    /**
     * 设置定时任务
     */
    setupTimers() {
        // 定期检查连接状态
        setInterval(() => {
            this.checkConnectionStatus();
        }, 5000);

        // 定期保存配置
        setInterval(() => {
            this.saveConfig();
        }, 30000);

        // 定期清理内存
        setInterval(() => {
            this.performMemoryCleanup();
        }, 60000);
    }

    /**
     * 设置键盘快捷键
     */
    setupKeyboardShortcuts() {
        document.addEventListener('keydown', (event) => {
            // Ctrl+Enter: 连接/断开
            if (event.ctrlKey && event.key === 'Enter') {
                event.preventDefault();
                this.toggleConnection();
            }
            
            // Ctrl+R: 刷新统计
            if (event.ctrlKey && event.key === 'r') {
                event.preventDefault();
                window.uiController.refreshStatistics();
            }
            
            // Ctrl+L: 清空日志
            if (event.ctrlKey && event.key === 'l') {
                event.preventDefault();
                window.uiController.clearLog();
            }
            
            // Ctrl+H: 获取历史
            if (event.ctrlKey && event.key === 'h') {
                event.preventDefault();
                window.uiController.getMessageHistory();
            }
            
            // Ctrl+S: 导出数据
            if (event.ctrlKey && event.key === 's') {
                event.preventDefault();
                window.uiController.exportData();
            }
        });
    }

    /**
     * 自动连接
     */
    autoConnect() {
        if (!window.mqttClient.isConnectedToMQTT()) {
            this.log('正在执行自动连接...', 'info');
            window.uiController.connectToMQTT();
        }
    }

    /**
     * 切换连接状态
     */
    toggleConnection() {
        if (window.mqttClient.isConnectedToMQTT()) {
            window.uiController.disconnectFromMQTT();
        } else {
            window.uiController.connectToMQTT();
        }
    }

    /**
     * 检查连接状态
     */
    checkConnectionStatus() {
        if (window.mqttClient.isConnectedToMQTT()) {
            // 检查是否有新消息
            const lastMessage = window.dataProcessor.lastMessageTime;
            if (lastMessage) {
                const timeSinceLastMessage = Date.now() - lastMessage.getTime();
                if (timeSinceLastMessage > 30000) { // 30秒内没有消息
                    this.log('警告: 30秒内没有收到新消息', 'warning');
                }
            }
        }
    }

    /**
     * 页面隐藏时的处理
     */
    onPageHidden() {
        // 暂停非关键的定时任务
        this.pauseNonCriticalTasks();
    }

    /**
     * 页面显示时的处理
     */
    onPageVisible() {
        // 恢复定时任务
        this.resumeNonCriticalTasks();
        
        // 刷新数据
        if (window.mqttClient.isConnectedToMQTT()) {
            window.uiController.refreshStatistics();
        }
    }

    /**
     * 页面卸载时的处理
     */
    onPageUnload() {
        // 保存配置
        this.saveConfig();
        
        // 断开MQTT连接
        if (window.mqttClient.isConnectedToMQTT()) {
            window.mqttClient.disconnect();
        }
    }

    /**
     * 窗口大小改变时的处理
     */
    onWindowResize() {
        // 可以在这里添加响应式处理逻辑
        this.log('窗口大小已改变', 'info');
    }

    /**
     * 暂停非关键任务
     */
    pauseNonCriticalTasks() {
        // 可以在这里实现暂停逻辑
    }

    /**
     * 恢复非关键任务
     */
    resumeNonCriticalTasks() {
        // 可以在这里实现恢复逻辑
    }

    /**
     * 执行内存清理
     */
    performMemoryCleanup() {
        // 清理过期的历史数据
        if (window.dataProcessor) {
            const history = window.dataProcessor.messageHistory;
            if (history.length > 50) {
                window.dataProcessor.messageHistory = history.slice(0, 50);
            }
        }
    }

    /**
     * 加载配置
     */
    loadConfig() {
        return {
            host: 'localhost',
            port: 8083,
            username: '',
            password: '',
            topicPrefix: 'ros2',
            autoConnect: false,
            maxHistorySize: 100,
            logLevel: 'info'
        };
    }

    /**
     * 保存配置
     */
    saveConfig() {
        try {
            // 从UI获取当前配置
            if (window.uiController && window.uiController.elements) {
                const elements = window.uiController.elements;
                this.config = {
                    ...this.config,
                    host: elements.mqttHost?.value || this.config.host,
                    port: parseInt(elements.mqttPort?.value) || this.config.port,
                    username: elements.mqttUsername?.value || this.config.username,
                    password: elements.mqttPassword?.value || this.config.password,
                    topicPrefix: elements.mqttTopicPrefix?.value || this.config.topicPrefix
                };
            }

            localStorage.setItem('mqtt-dashboard-config', JSON.stringify(this.config));
        } catch (error) {
            this.log(`保存配置失败: ${error.message}`, 'error');
        }
    }

    /**
     * 获取应用程序状态
     */
    getStatus() {
        return {
            initialized: this.initialized,
            connected: window.mqttClient?.isConnectedToMQTT() || false,
            config: this.config,
            statistics: window.dataProcessor?.getStatistics() || {}
        };
    }

    /**
     * 记录日志
     */
    log(message, level = 'info') {
        const timestamp = new Date().toLocaleTimeString();
        console.log(`[${timestamp}] [APP] [${level.toUpperCase()}] ${message}`);
        
        if (window.uiController) {
            window.uiController.addLogEntry(`[App] ${message}`, level);
        }
    }
}

// 创建并启动应用程序
window.addEventListener('DOMContentLoaded', () => {
    window.app = new App();
});

// 导出到全局
window.App = App;
