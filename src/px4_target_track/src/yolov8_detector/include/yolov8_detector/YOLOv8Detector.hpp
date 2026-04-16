#pragma once

// NumPy 2.x 兼容性处理
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#define PY_ARRAY_UNIQUE_SYMBOL yolov8_detector_ARRAY_API

#include <Python.h>
#include <numpy/arrayobject.h>

#include <vector>
#include <memory>
#include <string>
#include <deque>
#include <map>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

namespace yolov8_detector {

struct DetectionResult {
    int class_id;
    std::string class_name;
    float confidence;
    cv::Rect2f bbox;
    cv::Point2f center;
    int track_id;  // 用于多目标跟踪
};

struct TrackingTarget {
    int track_id;
    int class_id;  // 添加class_id字段，与DetectionResult对应
    std::string class_name;
    cv::Point2f center;
    cv::Rect2f bbox;
    float confidence;
    int frames_without_detection;  // 帧数统计
    cv::Point2f velocity;  // 速度估计用于卡尔曼滤波
};

class YOLOv8Detector {
public:
    YOLOv8Detector(const std::string& model_path, 
                   float confidence_threshold = 0.5f,
                   float nms_threshold = 0.45f);
    
    ~YOLOv8Detector();

    /**
     * @brief 检测图像中的目标
     * @param image 输入图像
     * @param results 检测结果列表
     * @param target_class 指定要检测的类别（-1表示检测所有）
     */
    void detect(const cv::Mat& image, 
                std::vector<DetectionResult>& results,
                int target_class = -1);

    /**
     * @brief 使用简单匹配进行多目标跟踪
     * @param current_detections 当前帧的检测结果
     * @param tracked_targets 输出跟踪的目标列表
     */
    void update_tracking(const std::vector<DetectionResult>& current_detections,
                        std::vector<TrackingTarget>& tracked_targets);

    /**
     * @brief 获取检测的类别名称
     */
    std::vector<std::string> get_class_names() const { return class_names_; }

    /**
     * @brief 设置置信度阈值
     */
    void set_confidence_threshold(float threshold) { 
        confidence_threshold_ = threshold; 
    }

    /**
     * @brief 设置NMS阈值
     */
    void set_nms_threshold(float threshold) { 
        nms_threshold_ = threshold; 
    }

    /**
     * @brief 获取当前跟踪的目标列表
     */
    const std::vector<TrackingTarget>& get_tracked_targets() const { 
        return tracked_targets_; 
    }

    /**
     * @brief 获取指定目标的轨迹（最近10个位置）
     */
    std::deque<cv::Point2f> get_track_trail(int track_id) const;

private:
    void initialize_python();
    void cleanup_python();
    PyObject* cvmat_to_numpy(const cv::Mat& mat);
    
    PyObject* py_detector_;
    float confidence_threshold_;
    float nms_threshold_;
    cv::Size input_size_;
    std::vector<std::string> class_names_;
    
    // 跟踪相关
    std::vector<TrackingTarget> tracked_targets_;
    int next_track_id_;
    std::map<int, std::deque<cv::Point2f>> track_trails_;  // 存储每个目标的轨迹
};

}  // namespace yolov8_detector