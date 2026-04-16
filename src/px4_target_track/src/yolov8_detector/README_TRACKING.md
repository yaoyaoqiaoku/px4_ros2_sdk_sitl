# YOLOv8 目标检测与跟踪系统改进

## 主要功能

### 1. **鼠标框选目标** 🖱️
- **左键拖动**：在图像上框选感兴趣的目标  
- **Ctrl+左键点击**：清除当前选择

### 2. **单目标跟踪**
- 框选目标后，系统自动优先跟踪该目标
- 被选中的目标用**红色框**标注（其他目标用绿色）
- 目标丢失时自动清除选择

### 3. **改进的可视化显示**
- **目标ID显示**：每个检测目标显示唯一的跟踪ID
- **置信度显示**：实时显示目标检测置信度
- **类别名称**：显示检测到的目标类别
- **中心点标记**：红色圆点显示目标中心
- **轨迹线条**：显示目标的运动轨迹（最近10个位置）

## 文件修改说明

### 修改的文件

#### 1. `yolov8_detector_node.cpp`
**新增功能：**
- 全局指针和鼠标回调机制
- 新的成员变量：
  - `selected_target_id_` - 选中的目标ID
  - `mouse_selecting_` - 框选状态标志
  - `selection_start_` / `selection_end_` - 框选区域坐标

**新增方法：**
- `static void mouse_callback()` - 静态鼠标回调函数
- `void handle_mouse_event()` - 处理各种鼠标事件
- `void select_target_by_region()` - 根据框选区域选择目标
- `void draw_detections_and_tracks()` - 绘制检测和跟踪结果

**改进的方法：**
- `void image_callback()` - 改进处理单目标发布逻辑

#### 2. `YOLOv8Detector.hpp`
**新增成员变量：**
- `std::map<int, std::deque<cv::Point2f>> track_trails_` - 存储轨迹信息

**新增公共方法：**
- `const std::vector<TrackingTarget>& get_tracked_targets() const` - 获取当前跟踪的目标列表
- `std::deque<cv::Point2f> get_track_trail(int track_id) const` - 获取特定目标的轨迹

#### 3. `YOLOv8Detector.cpp`
**改进内容：**
- 在 `update_tracking()` 方法中添加轨迹记录逻辑
- 实现 `get_track_trail()` 方法
- 自动清理已删除目标的轨迹记录

## 使用方法

### 运行节点
```bash
ros2 run yolov8_detector yolov8_detector_node
```

### 交互操作
1. **启动后**：输出窗口会显示实时的摄像头画面和检测结果
2. **框选目标**：
   - 在图像窗口中左键拖动鼠标框选目标
   - 框选完成后自动关联检测到的目标
3. **清除选择**：
   - 按住 Ctrl 并点击鼠标，清除当前选择
4. **退出程序**：
   - 按 ESC 键退出程序

## 参数配置

在 ROS 2 的参数服务器中可配置以下参数（参考 `launch` 文件）：

```yaml
yolov8_detector_node:
  ros__parameters:
    model_path: "yolov8n.pt"          # 模型文件路径
    confidence_threshold: 0.5          # 置信度阈值
    nms_threshold: 0.45               # NMS阈值
    target_class_id: -1               # 目标类别（-1为所有）
    fx: 1269.0                        # 相机焦距x
    fy: 1269.0                        # 相机焦距y
    cx: 640.0                         # 相机光心x
    cy: 360.0                         # 相机光心y
```

## 输出话题

- `/target_pose` - 发布选中目标的3D位姿信息（PoseStamped）
- `/yolo_detection_image` - 发布带有检测框和跟踪信息的图像（Image）

## 算法改进

### 目标关联
- 使用**中心距离匹配**算法
- 距离阈值：50像素
- 最小重叠比例（IoU）：0.3

### 轨迹管理
- 记录每个目标的最近10个位置
- 自动清除丢失超过10帧的目标
- 清除已删除目标的轨迹记录

### 目标框的颜色编码
- **红色框** - 当前选中的目标
- **绿色框** - 其他检测到的目标
- **蓝色框** - 当前鼠标框选区域

## 改进亮点

✅ **更好的用户交互** - 直观的鼠标框选界面  
✅ **清晰的目标标识** - 显示ID、置信度、类别名称  
✅ **可视化轨迹** - 清晰展示目标运动轨迹  
✅ **智能目标管理** - 自动清除丢失目标  
✅ **实时性能** - 高效的目标匹配算法  

## 后续优化方向

1. **卡尔曼滤波** - 改进目标位置预测
2. **深度学习特征匹配** - 使用Re-ID特征进行更精确的关联
3. **多相机协同** - 支持多个摄像头的目标跟踪
4. **轨迹预测** - 长期目标轨迹预测
5. **弱信号恢复** - 短期遮挡后的目标重连

