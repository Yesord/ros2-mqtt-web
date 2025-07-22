# MQTT图像接收器 - 压缩打包功能使用指南

## 🎯 新增功能

增强版的MQTT图像接收器现在支持**自动压缩打包**功能，可以将接收到的图片自动打包成ZIP文件，便于存储和传输。

## 🚀 使用方法

### 1. 基本使用（不启用压缩）
```bash
python3 mqtt_image_receiver.py
```

### 2. 启用自动压缩打包
```bash
# 每10张图片打包一次（默认）
python3 mqtt_image_receiver.py --enable-archive

# 自定义批次大小：每5张图片打包一次
python3 mqtt_image_receiver.py --enable-archive --archive-batch-size 5

# 指定压缩包保存目录
python3 mqtt_image_receiver.py --enable-archive --archive-dir my_archives

# 完整参数示例
python3 mqtt_image_receiver.py \
  --host localhost \
  --port 1883 \
  --save-dir received_images \
  --enable-archive \
  --archive-batch-size 20 \
  --archive-dir compressed_batches
```

## 📋 命令行参数

| 参数 | 说明 | 默认值 | 示例 |
|------|------|--------|------|
| `--host` | MQTT服务器地址 | `localhost` | `--host 192.168.1.100` |
| `--port` | MQTT服务器端口 | `1883` | `--port 8883` |
| `--save-dir` | 图像保存目录 | `mqtt_images` | `--save-dir my_images` |
| `--enable-archive` | 启用压缩打包 | 禁用 | `--enable-archive` |
| `--archive-batch-size` | 每包图片数量 | `10` | `--archive-batch-size 5` |
| `--archive-dir` | 压缩包目录 | `mqtt_archives` | `--archive-dir backups` |

## 📦 压缩打包功能特性

### 自动打包
- ✅ 达到指定数量时自动创建压缩包
- ✅ 退出时自动处理剩余文件
- ✅ 支持多种图片格式（JPG、PNG、BIN等）

### 压缩信息
- 📊 显示原始大小、压缩后大小
- 📊 计算压缩率百分比
- 📊 实时显示打包进度

### 文件命名
```
压缩包命名格式：mqtt_images_batch_序号_时间戳.zip
示例：mqtt_images_batch_0001_20250722_143256.zip
```

## 📁 目录结构示例

```
工作目录/
├── mqtt_images/                    # 原始图片目录
│   ├── ros2_image_compressed_data_20250722_143201_001_0001.jpg
│   ├── ros2_image_compressed_data_20250722_143202_002_0002.jpg
│   └── ...
└── mqtt_archives/                  # 压缩包目录
    ├── mqtt_images_batch_0001_20250722_143210.zip
    ├── mqtt_images_batch_0002_20250722_143220.zip
    └── ...
```

## 💻 运行示例

### 示例1：基本压缩功能
```bash
$ python3 mqtt_image_receiver.py --enable-archive --archive-batch-size 5
============================================================
MQTT图像接收器配置:
  MQTT服务器: localhost:1883
  图像保存目录: mqtt_images
  压缩打包: 启用
  批次大小: 5 张图片/包
  压缩包目录: mqtt_archives
============================================================
图像保存目录: /home/user/mqtt_images
压缩包保存目录: /home/user/mqtt_archives
压缩批次大小: 5 张图片/包
正在连接到 MQTT 服务器 localhost:1883...
✓ 已连接到MQTT服务器
已订阅话题: ros2/image_compressed/data

图像接收器已启动
等待图像数据...
按 Ctrl+C 退出
============================================================

[14:32:01] 收到消息:
  话题: ros2/image_compressed/data
  数据长度: 342156 字节
  消息类型: JSON
  发现uint8_array图像数据 (MQTT格式)
  数据大小: 256959 字节
  Base64数据长度: 342612 字符
  开始Base64解码...
  Base64解码成功，得到 256959 字节
  准备保存到: mqtt_images/ros2_image_compressed_data_20250722_143201_001_0001.jpg
  保存为压缩图像文件...
✓ 压缩图像已保存: ros2_image_compressed_data_20250722_143201_001_0001.jpg (256959 字节)
  添加到打包队列: ros2_image_compressed_data_20250722_143201_001_0001.jpg (队列中 1/5)

[... 更多图片接收 ...]

📦 创建压缩包: mqtt_images_batch_0001_20250722_143210.zip
   包含 5 个文件
   ✓ 已添加: ros2_image_compressed_data_20250722_143201_001_0001.jpg
   ✓ 已添加: ros2_image_compressed_data_20250722_143202_002_0002.jpg
   ✓ 已添加: ros2_image_compressed_data_20250722_143203_003_0003.jpg
   ✓ 已添加: ros2_image_compressed_data_20250722_143204_004_0004.jpg
   ✓ 已添加: ros2_image_compressed_data_20250722_143205_005_0005.jpg
   📊 压缩完成:
      原始大小: 1,284,795 字节
      压缩后: 1,278,234 字节
      压缩率: 0.5%
   💾 保存到: mqtt_archives/mqtt_images_batch_0001_20250722_143210.zip
```

### 示例2：处理剩余文件
```bash
^C
收到中断信号，正在退出...

📦 处理剩余的 3 个文件...
📦 创建压缩包: mqtt_images_batch_0002_20250722_143230.zip
   包含 3 个文件
   ✓ 已添加: ros2_image_compressed_data_20250722_143226_006_0006.jpg
   ✓ 已添加: ros2_image_compressed_data_20250722_143227_007_0007.jpg
   ✓ 已添加: ros2_image_compressed_data_20250722_143228_008_0008.jpg
   📊 压缩完成:
      原始大小: 770,877 字节
      压缩后: 766,540 字节
      压缩率: 0.6%
   💾 保存到: mqtt_archives/mqtt_images_batch_0002_20250722_143230.zip

接收会话结束，共保存 8 张图像
共创建 2 个压缩包
压缩包保存在: /home/user/mqtt_archives
```

## 🔧 高级功能

### 可选：删除已打包的原始文件
如果你想在创建压缩包后删除原始图片文件以节省空间，可以取消注释 `create_archive()` 方法中的这行代码：
```python
# self.cleanup_archived_files()  # 取消注释来启用
```

### 自定义压缩级别
当前使用 `zipfile.ZIP_DEFLATED` 和 `compresslevel=6`。可以调整以获得不同的压缩效果：
- `compresslevel=1`: 最快压缩，较大文件
- `compresslevel=9`: 最佳压缩，较慢速度

## 🛠️ 故障排除

### 常见问题

1. **压缩包创建失败**
   ```
   检查磁盘空间是否充足
   确认压缩包目录有写入权限
   ```

2. **压缩率很低**
   ```
   JPEG图片已经是压缩格式，再压缩效果有限
   这是正常现象，主要目的是批量打包管理
   ```

3. **内存使用过高**
   ```
   减少 --archive-batch-size 的值
   避免处理过大的单个图片文件
   ```

## 📊 性能考虑

- **批次大小建议**: 5-20张图片/包，平衡压缩效率和处理速度
- **磁盘空间**: 确保有足够空间存储原始文件和压缩包
- **内存使用**: 批量压缩时会暂时占用更多内存

## 🎯 使用场景

1. **长期数据收集**: 自动将图片分批打包，便于存档
2. **传输优化**: 压缩包更适合网络传输
3. **存储管理**: 减少文件数量，提高文件系统效率
4. **备份归档**: 按时间批次创建备份文件
