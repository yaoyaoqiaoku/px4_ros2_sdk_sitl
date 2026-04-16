#pragma once
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/core/quaternion.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <stdexcept>  // 异常处理
#include <string>     // 字符串处理

// ArUco跟踪器节点类：实现双ArUco标记（大/小码）的检测与位姿发布
class ArucoTrackerNode : public rclcpp::Node
{
public:
    /**
     * @brief 构造函数：初始化节点、加载参数、创建检测器/订阅者/发布者
     */
    ArucoTrackerNode();

private:
    /**
     * @brief 加载并校验ROS 2参数
     * @throw std::invalid_argument 参数非法时抛出异常
     */
    void loadParameters();

    /**
     * @brief 图像消息回调函数：处理图像、检测ArUco标记、计算并发布位姿
     * @param msg 原始图像消息
     */
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

    /**
     * @brief 相机内参回调函数：解析并存储相机内参和畸变系数
     * @param msg 相机内参消息
     */
    void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

    /**
     * @brief 在图像上标注ArUco标记的位姿信息（ID + X/Y/Z坐标）
     * @param image OpenCV图像指针
     * @param target 平移向量（X/Y/Z）
     * @param id ArUco标记ID
     */
    void annotate_image(cv_bridge::CvImagePtr image, const cv::Vec3d& target, int id);

    // ---------------- 成员变量 ----------------
    // 订阅者
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _image_sub;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr _camera_info_sub;

    // 发布者
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _image_pub;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _target_pose_large_pub;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _target_pose_small_pub;

    // ArUco检测器
    std::unique_ptr<cv::aruco::ArucoDetector> _detector;

    // 相机参数
    cv::Mat _camera_matrix;  // 内参矩阵 (3x3)
    cv::Mat _dist_coeffs;    // 畸变系数

    // ArUco配置参数
    int _param_aruco_id_large{49};    // 大码ID（默认49）
    int _param_aruco_id_small{50};    // 小码ID（默认50）
    int _param_dictionary{7};         // 字典类型（默认7=DICT_5X5_1000）
    double _param_marker_size_large{0.25};  // 大码尺寸（米，默认0.25）
    double _param_marker_size_small{0.036}; // 小码尺寸（米，默认0.036）

    // 常量定义（提升可读性）
    static constexpr int MIN_DICT_ID{0};    // 最小字典ID
    static constexpr int MAX_DICT_ID{20};   // 最大字典ID（对应DICT_APRILTAG_36H11）
    static constexpr double MIN_MARKER_SIZE{0.001}; // 最小标记尺寸（米）
};
