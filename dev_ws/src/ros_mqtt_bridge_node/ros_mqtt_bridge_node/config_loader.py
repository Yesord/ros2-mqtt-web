#!/usr/bin/env python3

import yaml
import os
from typing import Dict, List, Any, Optional
from rclpy.logging import get_logger


class BridgeConfigLoader:
    """桥接配置加载器"""
    
    def __init__(self, config_file_path: str = None):
        self.logger = get_logger('BridgeConfigLoader')
        self.config_file_path = config_file_path
        self.config_data = None
        
    def load_config(self, config_file_path: str = None) -> bool:
        """加载配置文件"""
        if config_file_path:
            self.config_file_path = config_file_path
            
        if not self.config_file_path:
            self.logger.error('未指定配置文件路径')
            return False
            
        try:
            # 检查文件是否存在
            if not os.path.exists(self.config_file_path):
                self.logger.error(f'配置文件不存在: {self.config_file_path}')
                return False
                
            # 加载YAML文件
            with open(self.config_file_path, 'r', encoding='utf-8') as file:
                self.config_data = yaml.safe_load(file)
                
            # 验证配置
            if not self._validate_config():
                return False
                
            self.logger.info(f'成功加载配置文件: {self.config_file_path}')
            return True
            
        except yaml.YAMLError as e:
            self.logger.error(f'解析YAML配置文件失败: {str(e)}')
            return False
        except Exception as e:
            self.logger.error(f'加载配置文件时发生错误: {str(e)}')
            return False
    
    def _validate_config(self) -> bool:
        """验证配置文件格式"""
        if not self.config_data:
            self.logger.error('配置数据为空')
            return False
            
        # 检查必需的顶级字段
        required_fields = ['mqtt_global', 'bridge_global', 'bridges']
        for field in required_fields:
            if field not in self.config_data:
                self.logger.error(f'配置文件缺少必需字段: {field}')
                return False
        
        # 验证桥接配置
        bridges = self.config_data.get('bridges', [])
        if not isinstance(bridges, list):
            self.logger.error('bridges字段必须是列表')
            return False
            
        for i, bridge in enumerate(bridges):
            if not self._validate_bridge_config(bridge, i):
                return False
                
        self.logger.info(f'配置验证通过，找到 {len(bridges)} 个桥接配置')
        return True
    
    def _validate_bridge_config(self, bridge: Dict, index: int) -> bool:
        """验证单个桥接配置"""
        required_fields = ['name', 'ros_config', 'mqtt_config']
        for field in required_fields:
            if field not in bridge:
                self.logger.error(f'桥接配置 {index} 缺少必需字段: {field}')
                return False
        
        # 验证ROS配置
        ros_config = bridge['ros_config']
        ros_required = ['topic', 'message_type', 'data_field']
        for field in ros_required:
            if field not in ros_config:
                self.logger.error(f'桥接 {bridge["name"]} 的ROS配置缺少字段: {field}')
                return False
        
        # 验证MQTT配置
        mqtt_config = bridge['mqtt_config']
        mqtt_required = ['topic_name', 'topic_suffix']
        for field in mqtt_required:
            if field not in mqtt_config:
                self.logger.error(f'桥接 {bridge["name"]} 的MQTT配置缺少字段: {field}')
                return False
                
        return True
    
    def get_mqtt_global_config(self) -> Dict[str, Any]:
        """获取全局MQTT配置"""
        if not self.config_data:
            return {}
        return self.config_data.get('mqtt_global', {})
    
    def get_bridge_global_config(self) -> Dict[str, Any]:
        """获取全局桥接配置"""
        if not self.config_data:
            return {}
        return self.config_data.get('bridge_global', {})
    
    def get_all_bridge_configs(self) -> List[Dict[str, Any]]:
        """获取所有桥接配置"""
        if not self.config_data:
            return []
        return self.config_data.get('bridges', [])
    
    def get_enabled_bridge_configs(self) -> List[Dict[str, Any]]:
        """获取启用的桥接配置"""
        all_bridges = self.get_all_bridge_configs()
        return [bridge for bridge in all_bridges if bridge.get('enabled', True)]
    
    def get_bridge_config_by_name(self, name: str) -> Optional[Dict[str, Any]]:
        """根据名称获取桥接配置"""
        all_bridges = self.get_all_bridge_configs()
        for bridge in all_bridges:
            if bridge.get('name') == name:
                return bridge
        return None
    
    def get_bridge_names(self) -> List[str]:
        """获取所有桥接名称"""
        all_bridges = self.get_all_bridge_configs()
        return [bridge.get('name', f'bridge_{i}') for i, bridge in enumerate(all_bridges)]
    
    def get_enabled_bridge_names(self) -> List[str]:
        """获取启用的桥接名称"""
        enabled_bridges = self.get_enabled_bridge_configs()
        return [bridge.get('name', f'bridge_{i}') for i, bridge in enumerate(enabled_bridges)]
    
    def is_bridge_enabled(self, name: str) -> bool:
        """检查桥接是否启用"""
        bridge = self.get_bridge_config_by_name(name)
        if bridge:
            return bridge.get('enabled', True)
        return False
    
    def get_config_version(self) -> str:
        """获取配置文件版本"""
        if not self.config_data:
            return "unknown"
        return self.config_data.get('version', 'unknown')
    
    def print_config_summary(self):
        """打印配置摘要"""
        if not self.config_data:
            self.logger.warning('未加载配置数据')
            return
            
        self.logger.info('=== 配置摘要 ===')
        self.logger.info(f'配置版本: {self.get_config_version()}')
        
        mqtt_config = self.get_mqtt_global_config()
        self.logger.info(f'MQTT服务器: {mqtt_config.get("broker_host")}:{mqtt_config.get("broker_port")}')
        
        all_bridges = self.get_all_bridge_configs()
        enabled_bridges = self.get_enabled_bridge_configs()
        
        self.logger.info(f'桥接总数: {len(all_bridges)}')
        self.logger.info(f'启用桥接: {len(enabled_bridges)}')
        
        for bridge in enabled_bridges:
            name = bridge.get('name')
            ros_topic = bridge.get('ros_config', {}).get('topic')
            mqtt_topic = f"{mqtt_config.get('topic_prefix')}/{bridge.get('mqtt_config', {}).get('topic_name')}/{bridge.get('mqtt_config', {}).get('topic_suffix')}"
            self.logger.info(f'  - {name}: {ros_topic} -> {mqtt_topic}')


def get_default_config_path() -> str:
    """获取默认配置文件路径"""
    try:
        # 首先尝试从ROS2包安装目录获取
        from ament_index_python.packages import get_package_share_directory
        package_share = get_package_share_directory('ros_mqtt_bridge_node')
        config_path = os.path.join(package_share, 'config', 'multi_bridge_config.yaml')
        
        if os.path.exists(config_path):
            return config_path
    except:
        # 如果ROS2包路径获取失败，使用开发环境路径
        pass
    
    # 开发环境：获取当前文件所在目录
    current_dir = os.path.dirname(os.path.abspath(__file__))
    # 上级目录的config文件夹
    config_dir = os.path.join(os.path.dirname(current_dir), 'config')
    config_path = os.path.join(config_dir, 'multi_bridge_config.yaml')
    
    if os.path.exists(config_path):
        return config_path
    
    # 如果开发环境路径也不存在，尝试src目录结构
    src_config_path = os.path.join(current_dir, '..', '..', 'config', 'multi_bridge_config.yaml')
    src_config_path = os.path.abspath(src_config_path)
    
    if os.path.exists(src_config_path):
        return src_config_path
    
    # 最后的备选方案：当前工作目录
    cwd_config_path = os.path.join(os.getcwd(), 'config', 'multi_bridge_config.yaml')
    if os.path.exists(cwd_config_path):
        return cwd_config_path
    
    # 如果都找不到，返回默认路径（可能不存在）
    return config_path


# 测试函数
def main():
    """测试配置加载器"""
    loader = BridgeConfigLoader()
    config_path = get_default_config_path()
    
    if loader.load_config(config_path):
        loader.print_config_summary()
    else:
        print("配置加载失败")


if __name__ == '__main__':
    main()
