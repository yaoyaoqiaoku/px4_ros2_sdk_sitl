#pragma once

#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/odometry/local_position.hpp>
#include <px4_ros2/odometry/attitude.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <cmath>
#include <Eigen/Dense>
#include <mutex>
#include <limits>

class TargetTrackerPlanner : public px4_ros2::ModeBase
{
public:
    explicit TargetTrackerPlanner(rclcpp::Node& node);

    // 回调函数
    void targetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    // 模式生命周期函数
    void onActivate() override;
    void onDeactivate() override;
    void updateSetpoint(float dt_s) override;

private:
    struct TargetPose {
        Eigen::Vector3d position = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
        Eigen::Quaterniond orientation;
        rclcpp::Time timestamp;

        bool valid() const {
            return !std::isnan(position.x()) && !std::isnan(position.y()) && !std::isnan(position.z());
        }
    };

    // 参数加载
    void loadParameters();

    // 坐标系转换函数
    // 从相机光学坐标系转换到NED世界坐标系
    TargetPose transformCameraToNED(const TargetPose& camera_target);

    // 状态机
    enum class State {
        Idle,		// 空闲
        Hovering,	// 悬停（目标丢失）
        Tracking,	// 跟踪目标
        Finished	// 完成
    };

    void switchToState(State state);
    std::string stateName(State state);

    // 辅助函数
    bool targetSeen(const TargetPose& target) const;
    bool targetFresh(const TargetPose& target, double timeout) const;
    Eigen::Vector2f calculateVelocitySetpointXY(const TargetPose& target, double dt);

    // ros2
    rclcpp::Node& _node;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr _target_pose_sub;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr _vehicle_status_sub;

    // px4接口对象
    std::shared_ptr<px4_ros2::OdometryLocalPosition> _vehicle_local_position;
    std::shared_ptr<px4_ros2::OdometryAttitude> _vehicle_attitude;
    std::shared_ptr<px4_ros2::TrajectorySetpointType> _trajectory_setpoint;

    // 状态变量
    State _state = State::Hovering;
    bool _mode_active = false;
    TargetPose _target_pose;		// NED坐标系中的目标位姿
    rclcpp::Time _target_last_update;

    // 参数
    float _param_max_horizontal_speed = 3.0f;
    float _param_max_vertical_speed = 1.0f;
    float _param_tracking_distance = 2.0f;
    float _param_yaw_rate = 0.5f;
    float _param_target_timeout = 3.0f;
    float _param_velocity_kp = 0.5f;
    float _param_velocity_kd = 0.1f;

    // 相机在机体坐标系中的偏置（实际应由标定给出）
    double _camera_offset_x = 0.0;		// X偏置（前）
    double _camera_offset_y = 0.0;		// Y偏置（右）
    double _camera_offset_z = -0.1;		// Z偏置（下）

    // 光学坐标系到NED坐标系的旋转矩阵
    // Optical: X right, Y down, Z away from lens
    // NED: X forward, Y right, Z down
    // 旋转关系：R = [[0, -1, 0], [1, 0, 0], [0, 0, 1]]
    Eigen::Quaterniond _quat_optical_to_ned;

    // 线程安全
    std::mutex _target_mutex;
    std::mutex _state_mutex;

    float _vel_x_integral = 0.0f;
    float _vel_y_integral = 0.0f;
    float _prev_yaw_rate = 0.0f;

    bool _target_lost_prev = true;
    px4_msgs::msg::VehicleStatus _current_vehicle_status;
    std::mutex _status_mutex;

    // 悬停时保存的位置
    Eigen::Vector3f _hover_position;
};