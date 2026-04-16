#include "target_tracker_planner/target_tracker_planner.hpp"

#include <px4_ros2/components/node_with_mode.hpp>
#include <px4_ros2/utils/geometry.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>


// -------------------------- 自定义模式定义 ------------------------
static const std::string kModeName = "TargetTrackerCustom";
static const bool kEnableDebugOutput = true;
using namespace px4_ros2::literals;
// -------------------------- 自定义模式定义 ------------------------

TargetTrackerPlanner::TargetTrackerPlanner(rclcpp::Node& node)
    : ModeBase(node, kModeName),
      _node(node)
{
    // 初始化光学坐标系到NED坐标系的旋转四元数
    // 转换矩阵 R:
    // R = [[0, -1, 0],
    //      [1,  0, 0],
    //      [0,  0, 1]]
    Eigen::Matrix3d R_optical_to_ned;
    R_optical_to_ned << 0, -1, 0,
                        1,  0, 0,
                        0,  0, 1;
    _quat_optical_to_ned = Eigen::Quaterniond(R_optical_to_ned);

    // 创建 setpoint 和状态封装对象
    _trajectory_setpoint = std::make_shared<px4_ros2::TrajectorySetpointType>(*this);
    _vehicle_local_position = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);
    _vehicle_attitude = std::make_shared<px4_ros2::OdometryAttitude>(*this);

    // 订阅目标位姿（来自相机传感器，初始在光学坐标系）
    _target_pose_sub = _node.create_subscription<geometry_msgs::msg::PoseStamped>(
        "/target_pose",
        rclcpp::QoS(1).best_effort(),
        std::bind(&TargetTrackerPlanner::targetPoseCallback, this, std::placeholders::_1));

    // 订阅VehicleStatus
    rclcpp::QoS px4_qos(10);
    px4_qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    px4_qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

    _vehicle_status_sub = _node.create_subscription<px4_msgs::msg::VehicleStatus>(
        "/fmu/out/vehicle_status",
        px4_qos,
        [this](const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(_status_mutex);
            _current_vehicle_status = *msg;
        }
    );

    // 读取参数
    loadParameters();

    // 指示该模式不需要手柄
    modeRequirements().manual_control = false;

    RCLCPP_INFO(_node.get_logger(), "TargetTrackerPlanner initialized!");
}

void TargetTrackerPlanner::loadParameters()
{
    // 声明参数
    _node.declare_parameter<float>("max_horizontal_speed", 3.0f);  // 最大水平速度
    _node.declare_parameter<float>("max_vertical_speed", 1.0f);    // 最大垂直速度
    _node.declare_parameter<float>("tracking_distance", 2.0f);     // 期望跟踪距离
    _node.declare_parameter<float>("yaw_rate", 0.5f);              // 期望偏航速率
    _node.declare_parameter<float>("target_timeout", 3.0f);        // 目标丢失超时时间
    _node.declare_parameter<float>("velocity_kp", 0.5f);           // 速度控制 P 增益
    _node.declare_parameter<float>("velocity_kd", 0.1f);           // 速度控制 D 增益
    
    // 相机安装参数（机体中心到相机光心的偏置）
    _node.declare_parameter<double>("camera_offset_x", 0.0);        // 相机在机体前后方向的偏置（正值表示相机在机体前方）
    _node.declare_parameter<double>("camera_offset_y", 0.0);        // 相机在机体左右方向的偏置（正值表示相机在机体右侧）
    _node.declare_parameter<double>("camera_offset_z", -0.1);       // 相机在机体垂直方向的偏置（正值表示相机在机体下方）

    // 读取参数
    _node.get_parameter("max_horizontal_speed", _param_max_horizontal_speed);
    _node.get_parameter("max_vertical_speed", _param_max_vertical_speed);
    _node.get_parameter("tracking_distance", _param_tracking_distance);
    _node.get_parameter("yaw_rate", _param_yaw_rate);
    _node.get_parameter("target_timeout", _param_target_timeout);
    _node.get_parameter("velocity_kp", _param_velocity_kp);
    _node.get_parameter("velocity_kd", _param_velocity_kd);
    _node.get_parameter("camera_offset_x", _camera_offset_x);
    _node.get_parameter("camera_offset_y", _camera_offset_y);
    _node.get_parameter("camera_offset_z", _camera_offset_z);

    // 打印关键参数
    RCLCPP_INFO(_node.get_logger(), "=== Target Tracker Parameters ===");
    RCLCPP_INFO(_node.get_logger(), "max_horizontal_speed: %.3f m/s", _param_max_horizontal_speed);
    RCLCPP_INFO(_node.get_logger(), "max_vertical_speed: %.3f m/s", _param_max_vertical_speed);
    RCLCPP_INFO(_node.get_logger(), "tracking_distance: %.3f m", _param_tracking_distance);
    RCLCPP_INFO(_node.get_logger(), "target_timeout: %.3f s", _param_target_timeout);
    RCLCPP_INFO(_node.get_logger(), "velocity_kp: %.3f", _param_velocity_kp);
    RCLCPP_INFO(_node.get_logger(), "velocity_kd: %.3f", _param_velocity_kd);
    RCLCPP_INFO(_node.get_logger(), "camera_offset (body frame): [%.3f, %.3f, %.3f] m", 
                _camera_offset_x, _camera_offset_y, _camera_offset_z);
    RCLCPP_INFO(_node.get_logger(), "================================");
}

void TargetTrackerPlanner::targetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    if (_mode_active) {
        std::lock_guard<std::mutex> lock(_target_mutex);
        
        // 从相机话题接收的目标位姿（初始在光学坐标系）
        TargetPose camera_target;
        camera_target.position = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
        camera_target.orientation = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, 
                                                    msg->pose.orientation.y, msg->pose.orientation.z);
        camera_target.timestamp = _node.now();

        // 转换到NED世界坐标系
        _target_pose = transformCameraToNED(camera_target);
        _target_last_update = _node.now();

        RCLCPP_DEBUG(_node.get_logger(), 
                     "Target received in camera frame: [%.3f, %.3f, %.3f] -> NED: [%.3f, %.3f, %.3f]",
                     camera_target.position.x(), camera_target.position.y(), camera_target.position.z(),
                     _target_pose.position.x(), _target_pose.position.y(), _target_pose.position.z());
    }
}

TargetTrackerPlanner::TargetPose TargetTrackerPlanner::transformCameraToNED(const TargetPose& camera_target)
{
    /**
     * 坐标系转换链：
     * camera_optical → camera_body → drone_body → world_NED
     * 
     * 1. camera_optical：相机光学坐标系
     *    - X: right, Y: down, Z: away from lens
     * 
     * 2. camera_body：相机在无人机机体坐标系中的位置和姿态
     *    - 偏置：(camera_offset_x, camera_offset_y, camera_offset_z)
     *    - 旋转：通过quat_optical_to_ned变换
     * 
     * 3. drone_body → world_NED：无人机位置和姿态
     *    - 位置：vehicle_local_position (NED)
     *    - 姿态：vehicle_attitude
     */

    // 获取无人机当前位姿（NED坐标系）
    auto vehicle_position = Eigen::Vector3d(_vehicle_local_position->positionNed().cast<double>());
    auto vehicle_orientation = Eigen::Quaterniond(_vehicle_attitude->attitude().cast<double>());

    // 步骤1：将目标位姿从光学相机坐标系旋转到机体坐标系
    Eigen::Affine3d optical_transform = Eigen::Translation3d(camera_target.position) * camera_target.orientation;

    // 步骤2：相机在机体中的坐标系偏置和旋转变换
    // 考虑相机相对于机体的安装位置和光学坐标系到机体坐标系的旋转
    Eigen::Affine3d camera_in_body = Eigen::Translation3d(_camera_offset_x, _camera_offset_y, _camera_offset_z) * _quat_optical_to_ned;

    // 步骤3：目标在机体坐标系中的位姿
    Eigen::Affine3d target_in_body = camera_in_body * optical_transform;

    // 步骤4：将机体坐标系转换到NED世界坐标系
    Eigen::Affine3d drone_transform = Eigen::Translation3d(vehicle_position) * vehicle_orientation;

    // 步骤5：目标在NED世界坐标系中的最终位姿
    Eigen::Affine3d target_world_transform = drone_transform * target_in_body;

    TargetPose ned_target;
    ned_target.position = target_world_transform.translation();
    ned_target.orientation = Eigen::Quaterniond(target_world_transform.rotation());
    ned_target.timestamp = camera_target.timestamp;

    return ned_target;
}

void TargetTrackerPlanner::onActivate()
{
    _mode_active = true;
    _vel_x_integral = 0.0f;
    _vel_y_integral = 0.0f;
    _prev_yaw_rate = 0.0f;
    _hover_position = _vehicle_local_position->positionNed();
    switchToState(State::Hovering);
    RCLCPP_INFO(_node.get_logger(), "Target Tracker activated, entering Hovering state at position [%.2f, %.2f, %.2f]",
                _hover_position.x(), _hover_position.y(), _hover_position.z());
}

void TargetTrackerPlanner::onDeactivate()
{
    _mode_active = false;
    RCLCPP_INFO(_node.get_logger(), "Target Tracker deactivated");
}

bool TargetTrackerPlanner::targetSeen(const TargetPose& target) const
{
    return target.valid();
}

bool TargetTrackerPlanner::targetFresh(const TargetPose& target, double timeout) const
{
    if (!targetSeen(target)) {
        return false;
    }
    double time_since_update = (_node.now() - target.timestamp).seconds();
    return time_since_update < timeout;
}

Eigen::Vector2f TargetTrackerPlanner::calculateVelocitySetpointXY(const TargetPose& target, double dt)
{
    auto current_pos = _vehicle_local_position->positionNed();
    
    // 计算NED坐标系中的偏差
    float dx = target.position.x() - current_pos.x();
    float dy = target.position.y() - current_pos.y();
    
    // P控制
    float vx = _param_velocity_kp * dx;
    float vy = _param_velocity_kp * dy;
    
    // 积分项（防止积分饱和）
    const float max_integral = 1.0f;
    // 修复：将 dt 显式转换为 float，确保运算结果类型一致
    float dt_float = static_cast<float>(dt);
    _vel_x_integral = std::max(-max_integral, std::min(max_integral, _vel_x_integral + dx * dt_float));
    _vel_y_integral = std::max(-max_integral, std::min(max_integral, _vel_y_integral + dy * dt_float));
    
    vx += _param_velocity_kd * _vel_x_integral;
    vy += _param_velocity_kd * _vel_y_integral;
    
    // 速度限制
    float horizontal_speed = std::sqrt(vx*vx + vy*vy);
    if (horizontal_speed > _param_max_horizontal_speed) {
        vx = vx / horizontal_speed * _param_max_horizontal_speed;
        vy = vy / horizontal_speed * _param_max_horizontal_speed;
    }
    
    RCLCPP_DEBUG(_node.get_logger(), 
                 "Velocity control: target_offset=[%.3f, %.3f], velocity=[%.3f, %.3f]",
                 dx, dy, vx, vy);
    
    return Eigen::Vector2f(vx, vy);
}


void TargetTrackerPlanner::switchToState(State state)
{
    std::lock_guard<std::mutex> lock(_state_mutex);
    if (_state != state) {
        RCLCPP_INFO(_node.get_logger(), "State transition: %s -> %s", 
                    stateName(_state).c_str(), stateName(state).c_str());
        _state = state;
    }
}

std::string TargetTrackerPlanner::stateName(State state)
{
    switch (state) {
        case State::Idle: return "Idle";
        case State::Hovering: return "Hovering";
        case State::Tracking: return "Tracking";
        case State::Finished: return "Finished";
        default: return "Unknown";
    }
}

void TargetTrackerPlanner::updateSetpoint(float dt_s)
{
    std::lock_guard<std::mutex> target_lock(_target_mutex);
    std::lock_guard<std::mutex> state_lock(_state_mutex);

    // 检查目标是否可见和新鲜
    bool target_available = targetFresh(_target_pose, _param_target_timeout);

    // 打印目标丢失/找回日志（仅边沿触发）
    if (target_available && _target_lost_prev) {
        RCLCPP_INFO(_node.get_logger(), "Target acquired at position [%.3f, %.3f, %.3f] (NED)",
                    _target_pose.position.x(), _target_pose.position.y(), _target_pose.position.z());
        _target_lost_prev = false;
    } else if (!target_available && !_target_lost_prev) {
        RCLCPP_WARN(_node.get_logger(), "Target lost, entering Hovering state at [%.3f, %.3f, %.3f] (NED)",
                    _hover_position.x(), _hover_position.y(), _hover_position.z());
        _target_lost_prev = true;
    }

    switch (_state) {
    case State::Idle: {
        break;
    }

    case State::Hovering: {
        // 原地悬停 - 发送零速度指令
        px4_ros2::TrajectorySetpoint setpoint;
        setpoint.withVelocity(Eigen::Vector3f::Zero());
        _trajectory_setpoint->update(setpoint);

        // 检查目标是否出现
        if (target_available) {
            _vel_x_integral = 0.0f;
            _vel_y_integral = 0.0f;
            switchToState(State::Tracking);
        }

        RCLCPP_DEBUG(_node.get_logger(), "Hovering at position (NED): [%.3f, %.3f, %.3f]",
                     _hover_position.x(), _hover_position.y(), _hover_position.z());

        break;
    }

    case State::Tracking: {
        // 检查目标是否丢失
        if (!target_available) {
            // 保存当前位置为悬停位置
            _hover_position = _vehicle_local_position->positionNed();
            switchToState(State::Hovering);
            break;
        }

        // 计算速度设定点（在NED坐标系中）
        Eigen::Vector2f vel = calculateVelocitySetpointXY(_target_pose, dt_s);

        // 垂直控制（向下降低固定速率，NED中Z向下为正）
        float vz = _param_max_vertical_speed;

        px4_ros2::TrajectorySetpoint setpoint;
        setpoint.withVelocity(Eigen::Vector3f(vel.x(), vel.y(), vz));
        _trajectory_setpoint->update(setpoint);

        auto current_pos = _vehicle_local_position->positionNed();
        RCLCPP_DEBUG(_node.get_logger(), 
                     "Tracking: current_pos=[%.3f, %.3f, %.3f] target=[%.3f, %.3f, %.3f] vel=[%.3f, %.3f, %.3f]",
                     current_pos.x(), current_pos.y(), current_pos.z(),
                     _target_pose.position.x(), _target_pose.position.y(), _target_pose.position.z(),
                     vel.x(), vel.y(), vz);

        break;
    }

    case State::Finished: {
        break;
    }
    }
}

// 主函数使用 NodeWithMode
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    // 修复：移除对私有成员_mode的访问，直接使用NodeWithMode对象即可
    auto node_with_mode = std::make_shared<px4_ros2::NodeWithMode<TargetTrackerPlanner>>("target_tracker_planner", kEnableDebugOutput);
    
    RCLCPP_INFO(node_with_mode->get_logger(), "Target Tracker Planner node started");

    rclcpp::spin(node_with_mode);
    rclcpp::shutdown();
    return 0;
}
