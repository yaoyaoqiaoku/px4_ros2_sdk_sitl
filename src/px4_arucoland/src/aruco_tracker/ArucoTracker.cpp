#include "ArucoTracker.hpp"
#include <sstream>
#include <iomanip>   // 格式化输出
#include <utility>   // std::pair

// 构造函数实现
ArucoTrackerNode::ArucoTrackerNode()
    : Node("aruco_tracker_node")
{
    try {
        // 1. 加载并校验参数
        loadParameters();
        RCLCPP_INFO(this->get_logger(), "参数加载完成:大码ID=%d(尺寸=%.3fm),小码ID=%d(尺寸=%.3fm)，字典类型=%d",
                    _param_aruco_id_large, _param_marker_size_large,
                    _param_aruco_id_small, _param_marker_size_small,
                    _param_dictionary);

        // 2. 初始化ArUco检测器
        auto detectorParams = cv::aruco::DetectorParameters();
        // 可选：优化检测器参数（提升检测鲁棒性）
        detectorParams.cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX; // 亚像素级角点优化
        detectorParams.adaptiveThreshWinSizeMin = 3;
        detectorParams.adaptiveThreshWinSizeMax = 23;

        // 校验字典类型有效性
        if (_param_dictionary < MIN_DICT_ID || _param_dictionary > MAX_DICT_ID) {
            throw std::invalid_argument("字典类型超出范围 [0,20]，当前值：" + std::to_string(_param_dictionary));
        }
        auto dictionary = cv::aruco::getPredefinedDictionary(_param_dictionary);
        _detector = std::make_unique<cv::aruco::ArucoDetector>(dictionary, detectorParams);

        // 3. 设置QoS（图像传输优先实时性，允许丢包）
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();

        // 4. 创建订阅者/发布者
        _image_sub = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera", qos, std::bind(&ArucoTrackerNode::image_callback, this, std::placeholders::_1));

        _camera_info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/camera_info", qos, std::bind(&ArucoTrackerNode::camera_info_callback, this, std::placeholders::_1));

        _image_pub = this->create_publisher<sensor_msgs::msg::Image>("/image_proc", qos);
        _target_pose_large_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("/target_pose_large", qos);
        _target_pose_small_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("/target_pose_small", qos);

        RCLCPP_INFO(this->get_logger(), "ArucoTracker节点初始化完成，等待图像和相机内参...");
    } catch (const std::exception& e) {
        RCLCPP_FATAL(this->get_logger(), "节点初始化失败：%s", e.what());
        rclcpp::shutdown(); // 初始化失败时退出
    }
}

// 参数加载与校验
void ArucoTrackerNode::loadParameters()
{
    // 声明参数（带默认值）
    this->declare_parameter<int>("aruco_id_large", _param_aruco_id_large);
    this->declare_parameter<int>("aruco_id_small", _param_aruco_id_small);
    this->declare_parameter<int>("dictionary", _param_dictionary);
    this->declare_parameter<double>("marker_size_large", _param_marker_size_large);
    this->declare_parameter<double>("marker_size_small", _param_marker_size_small);

    // 获取参数
    this->get_parameter("aruco_id_large", _param_aruco_id_large);
    this->get_parameter("aruco_id_small", _param_aruco_id_small);
    this->get_parameter("dictionary", _param_dictionary);
    this->get_parameter("marker_size_large", _param_marker_size_large);
    this->get_parameter("marker_size_small", _param_marker_size_small);

    // 参数校验
    if (_param_marker_size_large <= MIN_MARKER_SIZE || _param_marker_size_small <= MIN_MARKER_SIZE) {
        throw std::invalid_argument("标记尺寸必须大于" + std::to_string(MIN_MARKER_SIZE) + 
                                    "米，当前大码：" + std::to_string(_param_marker_size_large) + 
                                    "，小码：" + std::to_string(_param_marker_size_small));
    }
    if (_param_aruco_id_large < 0 || _param_aruco_id_small < 0) {
        throw std::invalid_argument("ArUco ID不能为负数，当前大码：" + std::to_string(_param_aruco_id_large) + 
                                    "，小码：" + std::to_string(_param_aruco_id_small));
    }
}

// 图像回调处理
void ArucoTrackerNode::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    try {
        // 1. ROS图像转OpenCV格式
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

        // 2. 检测ArUco标记
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
        _detector->detectMarkers(cv_ptr->image, corners, ids);
        cv::aruco::drawDetectedMarkers(cv_ptr->image, corners, ids); // 绘制标记边框

        // 3. 相机内参有效时计算位姿
        if (!_camera_matrix.empty() && !_dist_coeffs.empty() && _camera_matrix.rows == 3 && _camera_matrix.cols == 3) {
            // 3.1 角点去畸变
            std::vector<std::vector<cv::Point2f>> undistortedCorners;
            for (const auto& corner : corners) {
                std::vector<cv::Point2f> undistortedCorner;
                cv::undistortPoints(corner, undistortedCorner, _camera_matrix, _dist_coeffs, cv::noArray(), _camera_matrix);
                undistortedCorners.push_back(undistortedCorner);
            }

            // 3.2 遍历检测到的标记，计算目标ID的位姿
            for (size_t i = 0; i < ids.size(); ++i) {
                // 匹配目标ID，获取对应尺寸和发布者
                std::pair<double, rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> targetInfo;
                if (ids[i] == _param_aruco_id_large) {
                    targetInfo = {_param_marker_size_large, _target_pose_large_pub};
                } else if (ids[i] == _param_aruco_id_small) {
                    targetInfo = {_param_marker_size_small, _target_pose_small_pub};
                } else {
                    continue; // 忽略非目标ID
                }
                double markerSize = targetInfo.first;
                auto pub = targetInfo.second;

                // 3.3 定义标记3D模型点（中心为原点，右手系）
                const float halfSize = static_cast<float>(markerSize) / 2.0f;
                const std::vector<cv::Point3f> objectPoints = {
                    {-halfSize,  halfSize, 0.0f}, // 左上
                    { halfSize,  halfSize, 0.0f}, // 右上
                    { halfSize, -halfSize, 0.0f}, // 右下
                    {-halfSize, -halfSize, 0.0f}  // 左下
                };

                // 3.4 PnP求解位姿（使用SOLVEPNP_IPPE算法提升精度）
                cv::Vec3d rvec, tvec;
                bool pnpSuccess = cv::solvePnP(
                    objectPoints, undistortedCorners[i], _camera_matrix, cv::noArray(),
                    rvec, tvec, false, cv::SOLVEPNP_IPPE);

                if (!pnpSuccess) {
                    RCLCPP_WARN(this->get_logger(), "ID=%d的PnP位姿求解失败", ids[i]);
                    continue;
                }

                // 3.5 绘制坐标轴（可视化位姿）
                cv::drawFrameAxes(cv_ptr->image, _camera_matrix, cv::noArray(), rvec, tvec, markerSize);

                // 3.6 旋转向量转四元数（ROS标准）
                cv::Mat rotMat;
                cv::Rodrigues(rvec, rotMat);
                cv::Quatd quat = cv::Quatd::createFromRotMat(rotMat).normalize();

                // 3.7 构造并发布位姿消息
                geometry_msgs::msg::PoseStamped poseMsg;
                poseMsg.header.stamp = msg->header.stamp;
                poseMsg.header.frame_id = "camera_frame";
                poseMsg.pose.position.x = tvec[0];
                poseMsg.pose.position.y = tvec[1];
                poseMsg.pose.position.z = tvec[2];
                poseMsg.pose.orientation.x = quat.x;
                poseMsg.pose.orientation.y = quat.y;
                poseMsg.pose.orientation.z = quat.z;
                poseMsg.pose.orientation.w = quat.w;
                pub->publish(poseMsg);

                // 3.8 标注位姿信息到图像
                annotate_image(cv_ptr, tvec, ids[i]);
            }
        } else {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "相机内参未加载或无效，跳过位姿计算");
        }

        // 4. 发布处理后的图像
        _image_pub->publish(*cv_ptr->toImageMsg());

    } catch (const cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "CV_Bridge转换异常：%s", e.what());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "图像处理异常：%s", e.what());
    }
}

// 相机内参回调
void ArucoTrackerNode::camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
    try {
        // 校验内参矩阵有效性
        if (msg->k.size() != 9) {
            throw std::invalid_argument("相机内参矩阵维度错误，期望9个元素，实际：" + std::to_string(msg->k.size()));
        }

        // 深拷贝内参和畸变系数
        _camera_matrix = cv::Mat(3, 3, CV_64F, const_cast<double*>(msg->k.data())).clone();
        _dist_coeffs = cv::Mat(msg->d.size(), 1, CV_64F, const_cast<double*>(msg->d.data())).clone();

        // 校验焦距有效性
        double fx = _camera_matrix.at<double>(0, 0);
        double fy = _camera_matrix.at<double>(1, 1);
        if (fx <= 0 || fy <= 0) {
            throw std::invalid_argument("焦距无效：fx=" + std::to_string(fx) + ", fy=" + std::to_string(fy));
        }

        RCLCPP_INFO(this->get_logger(), "相机内参加载成功：fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f",
                    fx, fy,
                    _camera_matrix.at<double>(0, 2),
                    _camera_matrix.at<double>(1, 2));

        // 内参加载成功后取消订阅（只需一次）
        _camera_info_sub.reset();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "相机内参解析失败：%s", e.what());
    }
}

// 图像标注位姿信息
void ArucoTrackerNode::annotate_image(cv_bridge::CvImagePtr image, const cv::Vec3d& target, int id)
{
    // 格式化文本（保留2位小数）
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2)
        << "ID:" << id << " X:" << target[0] << " Y:" << target[1] << " Z:" << target[2];
    const std::string text = oss.str();

    // 文本样式配置
    const int fontFace = cv::FONT_HERSHEY_SIMPLEX;
    const double fontScale = 0.8;
    const int thickness = 2;
    const int lineType = cv::LINE_AA; // 抗锯齿

    // 动态调整文本位置（避免大/小码标注重叠）
    const int baseY = (id == _param_aruco_id_large) ? 30 : 60;
    const cv::Point textOrg(10, baseY);

    // 文本颜色（大码黄色，小码绿色）
    const cv::Scalar color = (id == _param_aruco_id_large) ? cv::Scalar(0, 255, 255) : cv::Scalar(0, 255, 0);

    // 绘制文本（加黑色描边提升可读性）
    cv::putText(image->image, text, textOrg, fontFace, fontScale, cv::Scalar(0, 0, 0), thickness + 1, lineType);
    cv::putText(image->image, text, textOrg, fontFace, fontScale, color, thickness, lineType);
}

// 主函数
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArucoTrackerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
