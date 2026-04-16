YOLOv8 无人机目标跟踪项目说明文档

项目简介
本项目基于 YOLOv8 模型实现目标检测 / 分割 / 分类等相关任务（可根据实际场景补充具体业务方向），为保证运行环境一致性，需使用指定虚拟环境 yolov8_env 进行代码运行。

环境准备
1. 激活虚拟环境
在运行项目代码前，需先激活预先配置好的 yolov8_env 虚拟环境
source yolov8_env/bin/activate
激活成功后，命令行前缀会出现 (yolov8_env) 标识，说明已进入指定虚拟环境

编译指定包
colcon build --packages-select yolov8_detector


退出虚拟环境
deactivate

代码文件说明

train.py	模型训练	基于自定义数据集 / 官方数据集训练 YOLOv8 模型，可配置训练轮数、批次大小、学习率、数据集路径等参数；运行命令：python train.py（需先修改配置参数）

detect.py	目标检测推理	加载训练好的模型权重，对图片 / 视频 / 摄像头流进行目标检测，输出检测结果（可视化标注 / 文本结果）；运行命令：python detect.py --weights 权重文件路径 --source 待检测文件路径

val.py	模型评估	对训练好的模型在验证集上进行精度评估（mAP、Precision、Recall 等指标），验证模型效果；运行命令：python val.py --weights 权重文件路径 --data 数据集配置文件

export.py	模型导出	将训练好的 YOLOv8 模型导出为 ONNX/TensorRT/ONNX 等格式，适配不同部署场景；运行命令：python export.py --weights 权重文件路径 --format 导出格式

utils.py	工具函数集	包含数据预处理（图片缩放 / 归一化）、结果可视化、日志记录、路径处理等通用工具函数，被其他核心文件调用，无需单独运行

config.py	配置文件	集中管理训练 / 推理 / 评估的超参数（如数据集路径、模型版本、训练参数等），修改后生效于所有依赖该文件的代码

2. 运行代码
确保已经创建好虚拟环境
source 