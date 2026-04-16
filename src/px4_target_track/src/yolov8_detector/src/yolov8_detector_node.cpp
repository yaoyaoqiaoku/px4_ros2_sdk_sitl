#define NO_IMPORT_ARRAY
#define PY_ARRAY_UNIQUE_SYMBOL yolov8_detector_ARRAY_API
#include <numpy/arrayobject.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "yolov8_detector/YOLOv8Detector.hpp"
#include <deque>

// 全局静态指针，用于鼠标回调函数
static YOLOv8DetectorNode* g_node_ptr = nullptr;

class YOLOv8DetectorNode : public rclcpp::Node {
public:
    YOLOv8DetectorNode()
        : Node("yolov8_detector_node"),
        // 初始化成员变量：检测器指针为空，相机内参矩阵为单位阵，畸变系数为零
          detector_(nullptr),
          camera_matrix_(cv::Mat::eye(3, 3, CV_32F)),
          dist_coeffs_(cv::Mat::zeros(4, 1, CV_32F)),
          selected_target_id_(-1),
          mouse_selecting_(false),
          selection_start_(0, 0),
          selection_end_(0, 0)
    {
        // 设置全局指针用于鼠标回调
        g_node_ptr = this;
        // --- 第一部分：参数声明与获取 ---
        // 从ROS参数服务器声明并读取一系列配置参数。
        // 包括：模型路径、置信度阈值、NMS阈值、要检测的特定类别ID、相机内参（焦距fx, fy和光心cx, cy）。
        this->declare_parameter<std::string>("model_path", "yolov8n.pt");
        this->declare_parameter<float>("confidence_threshold", 0.5f);
        this->declare_parameter<float>("nms_threshold", 0.45f);
        this->declare_parameter<int>("target_class_id", -1);  // -1表示检测所有目标
        this->declare_parameter<float>("fx", 1269.0f);
        this->declare_parameter<float>("fy", 1269.0f);
        this->declare_parameter<float>("cx", 640.0f);
        this->declare_parameter<float>("cy", 360.0f);

        // 获取参数
        std::string model_path = this->get_parameter("model_path").as_string();
        float conf_thresh = this->get_parameter("confidence_threshold").as_double();
        float nms_thresh = this->get_parameter("nms_threshold").as_double();
        target_class_id_ = this->get_parameter("target_class_id").as_int();

        // 相机内参
        float fx = this->get_parameter("fx").as_double();
        float fy = this->get_parameter("fy").as_double();
        float cx = this->get_parameter("cx").as_double();
        float cy = this->get_parameter("cy").as_double();

        RCLCPP_INFO(this->get_logger(), "Loaded camera intrinsics from parameter: fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f", fx, fy, cx, cy);

        camera_matrix_ = (cv::Mat_<float>(3, 3) << 
            fx, 0, cx,
            0, fy, cy,
            0, 0, 1);

        // --- 第二部分：YOLOv8检测器初始化 ---
        try {
            // 创建YOLOv8Detector类的实例，传入模型路径和阈值。
            detector_ = std::make_unique<yolov8_detector::YOLOv8Detector>(
                model_path, conf_thresh, nms_thresh);
            RCLCPP_INFO(this->get_logger(), "YOLOv8 detector initialized with model: %s", model_path.c_str());
        } catch (const std::exception& e) {
            // 如果初始化失败（例如模型文件不存在），记录错误并关闭ROS 2。
            RCLCPP_FATAL(this->get_logger(), "Failed to initialize YOLOv8 detector: %s", e.what());
            rclcpp::shutdown();
            return;
        }

        // --- 第三部分：ROS 2通信设置 ---
        // 设置服务质量(QoS)：只保留最新的1条消息，尽力而为传输，非持久化。
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();

        // 创建订阅者：订阅名为“/camera”的传感器图像话题，收到消息后调用`image_callback`函数。
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera", qos,
            std::bind(&YOLOv8DetectorNode::image_callback, this, std::placeholders::_1));

        // 发布检测到的目标位置
        target_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/target_pose", qos);

        // 发布处理后的图像（可选）
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/yolo_detection_image", qos);

        // 创建输出窗口
        cv::namedWindow("YOLOv8 Tracking", cv::WINDOW_AUTOSIZE);
        cv::setMouseCallback("YOLOv8 Tracking", mouse_callback, this);

        RCLCPP_INFO(this->get_logger(), "YOLOv8 detector node ready!");
        RCLCPP_INFO(this->get_logger(), "Use mouse to select target: left-click drag to select, double-click to clear");
    }

private:
    // 静态鼠标回调函数
    static void mouse_callback(int event, int x, int y, int flags, void* userdata) {
        if (g_node_ptr == nullptr) return;
        g_node_ptr->handle_mouse_event(event, x, y, flags);
    }

    // 处理鼠标事件
    void handle_mouse_event(int event, int x, int y, int flags) {
        if (event == cv::EVENT_LBUTTONDOWN) {
            // 开始框选
            if (flags & cv::EVENT_FLAG_CTRLKEY) {
                // 按住Ctrl+左键清除选择
                selected_target_id_ = -1;
                RCLCPP_INFO(this->get_logger(), "Selection cleared");
            } else {
                mouse_selecting_ = true;
                selection_start_ = cv::Point(x, y);
                selection_end_ = cv::Point(x, y);
            }
        } else if (event == cv::EVENT_MOUSEMOVE && mouse_selecting_) {
            // 实时更新框选区域
            selection_end_ = cv::Point(x, y);
        } else if (event == cv::EVENT_LBUTTONUP && mouse_selecting_) {
            // 结束框选，在所有检测到的目标中找最匹配的
            mouse_selecting_ = false;
            cv::Rect selection(
                std::min(selection_start_.x, selection_end_.x),
                std::min(selection_start_.y, selection_end_.y),
                std::abs(selection_end_.x - selection_start_.x),
                std::abs(selection_end_.y - selection_start_.y)
            );
            
            if (selection.width > 10 && selection.height > 10) {
                select_target_by_region(selection);
            }
        }
    }

    // 根据框选区域选择目标
    void select_target_by_region(const cv::Rect& selection) {
        float max_overlap = 0.3f;  // 最小重叠比例
        int best_target_id = -1;
        
        for (const auto& target : detector_->get_tracked_targets()) {
            cv::Rect target_rect = cv::Rect(
                target.bbox.x, target.bbox.y, 
                target.bbox.width, target.bbox.height
            );
            
            cv::Rect intersection = selection & target_rect;
            float overlap = static_cast<float>(intersection.area()) / 
                          (selection.area() + target_rect.area() - intersection.area());
            
            if (overlap > max_overlap) {
                max_overlap = overlap;
                best_target_id = target.track_id;
            }
        }
        
        if (best_target_id >= 0) {
            selected_target_id_ = best_target_id;
            RCLCPP_INFO(this->get_logger(), "Target selected: ID=%d", selected_target_id_);
        } else {
            RCLCPP_WARN(this->get_logger(), "No target found in selected region");
            selected_target_id_ = -1;
        }
    }

    // 图像回调函数：每当从相机话题收到新图像时被调用。
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            // 1. 格式转换：将ROS图像消息转换为OpenCV的Mat格式（BGR8颜色编码）。
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

            // 2. 执行检测：调用检测器，在当前帧中寻找目标。
            std::vector<yolov8_detector::DetectionResult> detections;
            detector_->detect(cv_ptr->image, detections, target_class_id_);

            // 3. 更新跟踪：将当前帧的检测结果与历史轨迹关联，实现简单的跨帧跟踪。
            std::vector<yolov8_detector::TrackingTarget> tracked_targets;
            detector_->update_tracking(detections, tracked_targets);

            // 4. 选择要发布的目标
            yolov8_detector::TrackingTarget* published_target = nullptr;
            
            if (selected_target_id_ >= 0) {
                // 优先发布选中的目标
                for (auto& target : tracked_targets) {
                    if (target.track_id == selected_target_id_) {
                        published_target = &target;
                        break;
                    }
                }
                
                // 如果选中的目标丢失，自动清除选择
                if (published_target == nullptr) {
                    RCLCPP_WARN(this->get_logger(), "Selected target lost, clearing selection");
                    selected_target_id_ = -1;
                }
            }
            
            // 如果没有选中目标，选择置信度最高的目标
            if (published_target == nullptr && !tracked_targets.empty()) {
                published_target = &tracked_targets[0];
                for (auto& target : tracked_targets) {
                    if (target.confidence > published_target->confidence) {
                        published_target = &target;
                    }
                }
            }

            // 5. 发布目标位置
            if (published_target != nullptr) {
                // 转换2D像素坐标到3D相机坐标
                geometry_msgs::msg::PoseStamped pose;
                pose.header.stamp = this->get_clock()->now();
                pose.header.frame_id = "camera";

                float depth = 2.0f;  // 假设目标距相机2米
                float fx = camera_matrix_.at<float>(0, 0);
                float fy = camera_matrix_.at<float>(1, 1);
                float cx = camera_matrix_.at<float>(0, 2);
                float cy = camera_matrix_.at<float>(1, 2);

                float x = (published_target->center.x - cx) * depth / fx;
                float y = (published_target->center.y - cy) * depth / fy;
                float z = depth;

                pose.pose.position.x = x;
                pose.pose.position.y = y;
                pose.pose.position.z = z;
                pose.pose.orientation.w = 1.0;

                target_pose_pub_->publish(pose);
            }

            // 6. 绘制并发布检测和跟踪结果的可视化图像
            draw_detections_and_tracks(cv_ptr->image, detections, tracked_targets);
            image_pub_->publish(*cv_ptr->toImageMsg());
            
            // 7. 显示到窗口（用于调试）
            cv::imshow("YOLOv8 Tracking", cv_ptr->image);
            int key = cv::waitKey(1);
            if (key == 27) {  // ESC键退出
                rclcpp::shutdown();
            }

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error in image callback: %s", e.what());
        }
    }

    // 绘制检测和跟踪结果
    void draw_detections_and_tracks(cv::Mat& image, 
                                     const std::vector<yolov8_detector::DetectionResult>& detections,
                                     const std::vector<yolov8_detector::TrackingTarget>& tracked_targets) {
        // 绘制所有跟踪的目标
        for (const auto& target : tracked_targets) {
            cv::Scalar color;
            
            // 选中的目标用红色，其他目标用绿色
            if (selected_target_id_ == target.track_id) {
                color = cv::Scalar(0, 0, 255);  // 红色
            } else {
                color = cv::Scalar(0, 255, 0);  // 绿色
            }
            
            // 绘制目标轨迹
            auto trail = detector_->get_track_trail(target.track_id);
            if (trail.size() > 1) {
                for (size_t i = 1; i < trail.size(); ++i) {
                    cv::line(image, 
                            cv::Point(trail[i-1].x, trail[i-1].y),
                            cv::Point(trail[i].x, trail[i].y),
                            color, 1);
                }
            }
            
            // 绘制边界框
            cv::Rect bbox(target.bbox.x, target.bbox.y, 
                         target.bbox.width, target.bbox.height);
            cv::rectangle(image, bbox, color, 2);
            
            // 绘制跟踪ID和置信度
            std::string label = "ID:" + std::to_string(target.track_id) + 
                              " " + target.class_name + 
                              " " + std::to_string(target.confidence).substr(0, 4);
            cv::putText(image, label,
                       cv::Point(bbox.x, bbox.y - 10),
                       cv::FONT_HERSHEY_SIMPLEX, 0.6, color, 2);
            
            // 绘制中心点
            cv::circle(image, cv::Point(target.center.x, target.center.y), 5, color, -1);
        }
        
        // 绘制当前的鼠标框选区域
        if (mouse_selecting_) {
            cv::rectangle(image, selection_start_, selection_end_, cv::Scalar(255, 0, 0), 2);
        }
    }

    // 成员变量
    std::unique_ptr<yolov8_detector::YOLOv8Detector> detector_; // YOLOv8检测器核心
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    cv::Mat camera_matrix_; // 相机内参矩阵
    cv::Mat dist_coeffs_;   // 相机畸变系数（本代码中未使用）
    int target_class_id_;   // 指定要检测的类别ID，-1表示检测所有类别
    
    // 目标选择相关成员变量
    int selected_target_id_;      // 选中的目标ID
    bool mouse_selecting_;        // 是否正在进行鼠标框选
    cv::Point selection_start_;   // 框选开始点
    cv::Point selection_end_;     // 框选结束点
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<YOLOv8DetectorNode>());
    rclcpp::shutdown();
    return 0;
}