#include "PrecisionLand.hpp"

#include <px4_ros2/components/node_with_mode.hpp>
#include <px4_ros2/utils/geometry.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

// -------------------------- 自定义模式定义 ------------------------
// 模式名称：用于PX4识别该自定义模式
static const std::string kModeName = "PrecisionLandCustom";
// 调试输出开关：true表示启用调试日志
static const bool kEnableDebugOutput = true;

// 引入px4_ros2命名空间的字面量（简化单位相关代码，如时间单位等）
using namespace px4_ros2::literals;

PrecisionLand::PrecisionLand(rclcpp::Node& node)
	: ModeBase(node, kModeName)	// 继承ModeBase基类，传入节点和模式名称
	, _node(node)
	, _last_debug_print_time(0.0)  // 新增：初始化调试打印时间戳
//-------------------------- 自定义模式定义 ------------------------

{
	// 创建 setpoint / 状态封装对象（与 px4_ros2 框架交互）
	_trajectory_setpoint = std::make_shared<px4_ros2::TrajectorySetpointType>(*this);
	_vehicle_local_position = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);
	_vehicle_attitude = std::make_shared<px4_ros2::OdometryAttitude>(*this);

	// 订阅视觉目标（Aruco）位姿，QoS 使用 best_effort（丢包可接受）,订阅大码位姿
	_target_pose_large_sub = _node.create_subscription<geometry_msgs::msg::PoseStamped>("/target_pose_large",
			   rclcpp::QoS(1).best_effort(), std::bind(&PrecisionLand::targetPoseCallback, this, std::placeholders::_1));

	// 新增：订阅小码位姿
    _target_pose_small_sub = _node.create_subscription<geometry_msgs::msg::PoseStamped>("/target_pose_small",
               rclcpp::QoS(1).best_effort(), std::bind(&PrecisionLand::targetPoseSmallCallback, this, std::placeholders::_1));

	// 新增：创建vehicle_command发布者（QoS使用默认可靠传输，确保命令被PX4接收）
    _vehicle_command_pub = _node.create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command",
				rclcpp::QoS(10));
    			RCLCPP_INFO(_node.get_logger(), "成功创建vehicle_command发布者");

	// 核心修复：订阅PX4状态（使用自定义订阅者_vehicle_status_sub_custom）
	rclcpp::QoS px4_qos(10);
	px4_qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
	px4_qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

	_vehicle_status_sub_custom = _node.create_subscription<px4_msgs::msg::VehicleStatus>(
		"/fmu/out/vehicle_status",
		px4_qos,
		[this](const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
			std::lock_guard<std::mutex> lock(state_mutex_);
			current_vehicle_status_ = *msg;
		}
	);

	// 订阅飞控的降落检测信息
	_vehicle_land_detected_sub = _node.create_subscription<px4_msgs::msg::VehicleLandDetected>("/fmu/out/vehicle_land_detected",
				     rclcpp::QoS(1).best_effort(), std::bind(&PrecisionLand::vehicleLandDetectedCallback, this, std::placeholders::_1));

	// 新增：订阅激光高度数据
	// 新增：订阅激光高度话题
	_laser_height_sub = _node.create_subscription<px4_msgs::msg::DistanceSensor>(
		"/fmu/out/distance_sensor",
		rclcpp::QoS(10).best_effort(),
		std::bind(&PrecisionLand::laserHeightCallback, this, std::placeholders::_1)
	);
	if (_laser_height_sub) { // 判断订阅者是否创建成功
	RCLCPP_INFO(_node.get_logger(), "成功订阅激光高度话题 /fmu/out/distance_sensor");
		} else {
			RCLCPP_ERROR(_node.get_logger(), "激光高度话题订阅失败！");
		}
	// 读取参数（默认值在 loadParameters 中声明）
	loadParameters();

	// 指示该模式不需要手柄（manual_control）
	modeRequirements().manual_control = false;
}

void PrecisionLand::loadParameters()
{
	// 声明参数并设置默认值
	_node.declare_parameter<float>("descent_vel", 1.0);		//下降速度
	_node.declare_parameter<float>("vel_p_gain", 1.5);		//速度P增益
	_node.declare_parameter<float>("vel_i_gain", 0.0);		//速度I增益
	// 新增：相机在机体坐标系下的偏置参数（可通过参数文件配置）
	_node.declare_parameter<double>("camera_offset_x", 0.0);
	_node.declare_parameter<double>("camera_offset_y", 0.0);
	_node.declare_parameter<double>("camera_offset_z", -0.1);
	_node.declare_parameter<float>("max_velocity", 0.5);		//最大速度
	_node.declare_parameter<float>("target_timeout", 3.0);		//目标超时
	_node.declare_parameter<float>("delta_position", 0.25);		//位置到达阈值
	_node.declare_parameter<float>("delta_velocity", 0.25);		//速度到达阈值
	// 新增参数声明（可选，从配置文件读取）
    _node.declare_parameter<int>("aruco_id_large", 49);			// 大码 ID
    _node.declare_parameter<int>("aruco_id_small", 50);		// 小码 ID
    _node.declare_parameter<double>("height_threshold_switch", 2.0);	// 切换高度阈值
	_node.declare_parameter<double>("height_threshold_low", 0.2);      // 最终精控高度
    _node.declare_parameter<double>("height_threshold_recover", 2.5);  // 小码丢失回升高度
	_node.declare_parameter<double>("land_mode_switch_height", 2.0); // 新增参数声明
	// 新增：高度融合参数声明
	_node.declare_parameter<double>("param_visual_weight", 0.5);    // 视觉高度权重
	_node.declare_parameter<double>("param_laser_weight", 0.5);     // 激光高度权重

	// 新增：丢失确认与回升步长参数
	_node.declare_parameter<double>("small_loss_confirm_time", 0.3); // 丢失确认时间（秒）
	_node.declare_parameter<double>("recover_step_height", 0.5); // 每次回升步长（米）
	// 新增：Yaw轴最大偏航角速度
	_node.declare_parameter<float>("max_yaw_speed", 0.3f);  		// 默认值0.3 rad/s
	_node.declare_parameter<float>("yaw_kp", 1.0f);               // P 增益（可调）
	_node.declare_parameter<float>("yaw_deadband", 0.02f);        // 死区 (rad)，约 1.1 deg
	_node.declare_parameter<float>("yaw_alpha", 1.0f);            // 输出平滑系数 alpha in [0,1], 1.0 表示无平滑

	// 读取参数到成员变量
	_node.get_parameter("descent_vel", _param_descent_vel);
	_node.get_parameter("vel_p_gain", _param_vel_p_gain);
	_node.get_parameter("vel_i_gain", _param_vel_i_gain);
	_node.get_parameter("max_velocity", _param_max_velocity);
	_node.get_parameter("target_timeout", _param_target_timeout);
	_node.get_parameter("delta_position", _param_delta_position);
	_node.get_parameter("delta_velocity", _param_delta_velocity);
	// 新增：读取双码参数
	_node.get_parameter("aruco_id_large", _param_aruco_id_large);
	_node.get_parameter("aruco_id_small", _param_aruco_id_small);
	_node.get_parameter("height_threshold_switch", _height_threshold_switch);
	_node.get_parameter("height_threshold_low", _height_threshold_low);
	_node.get_parameter("height_threshold_recover", _height_threshold_recover);
	_node.get_parameter("land_mode_switch_height", _land_mode_switch_height); // 新增参数读取
	// 新增：读取高度融合参数
	_node.get_parameter("param_visual_weight", _param_visual_weight);
	_node.get_parameter("param_laser_weight", _param_laser_weight);

	// 新增：读取小码丢失确认与回升步长
	_node.get_parameter("small_loss_confirm_time", _param_small_loss_confirm_time);
	_node.get_parameter("recover_step_height", _param_recover_step_height);
	// 新增：读取Yaw轴最大偏航角速度
	_node.get_parameter("max_yaw_speed", _max_yaw_speed);
	_node.get_parameter("yaw_kp", _yaw_kp);
	_node.get_parameter("yaw_deadband", _yaw_deadband);
	_node.get_parameter("yaw_alpha", _yaw_alpha);
	// 读取相机偏置参数
	_node.get_parameter("camera_offset_x", _camera_offset_x);
	_node.get_parameter("camera_offset_y", _camera_offset_y);
	_node.get_parameter("camera_offset_z", _camera_offset_z);

	// 打印关键参数，方便调试
	RCLCPP_INFO(_node.get_logger(), "descent_vel: %f", _param_descent_vel);
	RCLCPP_INFO(_node.get_logger(), "vel_i_gain: %f", _param_vel_i_gain);
	RCLCPP_INFO(_node.get_logger(), "vel_p_gain: %f", _param_vel_p_gain);
	RCLCPP_INFO(_node.get_logger(), "max_velocity: %f", _param_max_velocity);
	RCLCPP_INFO(_node.get_logger(), "target_timeout: %f", _param_target_timeout);
	RCLCPP_INFO(_node.get_logger(), "delta_position: %f", _param_delta_position);
	RCLCPP_INFO(_node.get_logger(), "delta_velocity: %f", _param_delta_velocity);
	RCLCPP_INFO(_node.get_logger()," aruco_id_large: %d", _param_aruco_id_large);
	RCLCPP_INFO(_node.get_logger()," aruco_id_small: %d", _param_aruco_id_small);
	RCLCPP_INFO(_node.get_logger()," height_threshold_switch: %f", _height_threshold_switch);
	RCLCPP_INFO(_node.get_logger()," height_threshold_low: %f", _height_threshold_low);
	RCLCPP_INFO(_node.get_logger()," height_threshold_recover: %f", _height_threshold_recover);
	RCLCPP_INFO(_node.get_logger()," land_mode_switch_height: %f", _land_mode_switch_height);
	RCLCPP_INFO(_node.get_logger()," param_visual_weight: %f", _param_visual_weight);
	RCLCPP_INFO(_node.get_logger()," param_laser_weight: %f", _param_laser_weight);
	RCLCPP_INFO(_node.get_logger()," small_loss_confirm_time: %f", _param_small_loss_confirm_time);
	RCLCPP_INFO(_node.get_logger()," recover_step_height: %f", _param_recover_step_height);
	RCLCPP_INFO(_node.get_logger()," max_yaw_speed: %f", _max_yaw_speed);
	RCLCPP_INFO(_node.get_logger()," yaw_kp: %f", _yaw_kp);
	RCLCPP_INFO(_node.get_logger()," yaw_deadband: %f", _yaw_deadband);
	RCLCPP_INFO(_node.get_logger()," yaw_alpha: %f", _yaw_alpha);
	RCLCPP_INFO(_node.get_logger(), "camera_offset_x: %f", _camera_offset_x);
	RCLCPP_INFO(_node.get_logger(), "camera_offset_y: %f", _camera_offset_y);
	RCLCPP_INFO(_node.get_logger(), "camera_offset_z: %f", _camera_offset_z);
}

void PrecisionLand::vehicleLandDetectedCallback(const px4_msgs::msg::VehicleLandDetected::SharedPtr msg)
{
	// 更新降落检测标志（来自飞控）
	_land_detected = msg->landed;
}

void PrecisionLand::targetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
	// 只有当搜索已开始时才处理视觉消息，防止激活前误处理
	if (_search_started) {
		auto tag = ArucoTag {
			.position = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z),
			// 注意 PoseStamped 的四元数顺序：w, x, y, z
			.orientation = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z),
			.timestamp = _node.now(),
		};

		// 新增：保存原始相机系 Z 高度（线程安全）
        {
            std::lock_guard<std::mutex> lock(_raw_height_mutex);
            _large_tag_raw_z = msg->pose.position.z;  // 直接保存话题中的原始 z（正确高度）
        }

		// 将 tag（光学相机坐标）转换为 world（NED）坐标
		_tag_large = getTagWorld(tag);
	}

}

// yaw 控制相关：计算目标 yaw 速率
static inline float normalizeAngle(float a)
{
    while (a > M_PI) a -= 2.0f * M_PI;
    while (a < -M_PI) a += 2.0f * M_PI;
    return a;
}

/**
 * 计算对准目标航向的期望偏航角速度（rad/s）
 * current_yaw: 当前飞机朝向（rad）
 * target_yaw:  目标朝向（来自二维码）（rad）
 * max_yaw_rate: 允许的最大偏航速度（rad/s），使用仓库中已有的 _max_yaw_speed
 * kp: P 增益
 * deadband: 角度小于此值则返回 0
 * prev_rate: 上一次输出（用于平滑）
 * alpha: 平滑系数，1.0 表示不平滑
 */
static float computeYawRateToAlign(float current_yaw,
                                   float target_yaw,
                                   float max_yaw_rate,
                                   float kp = 1.0f,
                                   float deadband = 0.02f,
                                   float prev_rate = 0.0f,
                                   float alpha = 1.0f)
{
    float err = normalizeAngle(target_yaw - current_yaw);

    if (std::fabs(err) < deadband) {
        return 0.0f;
    }

    float rate = kp * err;

    if (rate > max_yaw_rate) rate = max_yaw_rate;
    if (rate < -max_yaw_rate) rate = -max_yaw_rate;

    if (alpha < 1.0f) {
        rate = prev_rate * (1.0f - alpha) + rate * alpha;
    }

    return rate;
}

// 新增：小码位姿回调函数
void PrecisionLand::targetPoseSmallCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
	// 只有当搜索已开始时才处理视觉消息，防止激活前误处理
	if (_search_started) {
		auto tag = ArucoTag {
			.position = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z),
			// 注意 PoseStamped 的四元数顺序：w, x, y, z
			.orientation = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z),
			.timestamp = _node.now(),
		};

		// 新增：保存原始相机系 Z 高度（线程安全）
        {
            std::lock_guard<std::mutex> lock(_raw_height_mutex);
            _small_tag_raw_z = msg->pose.position.z;  // 直接保存话题中的原始 z（正确高度）
        }

		// 将 tag（光学相机坐标）转换为 world（NED）坐标
		_tag_small = getTagWorld(tag);

		//小码找回，重置丢失状态
		if (_small_tag_lost) {
			_small_tag_lost = false;
			RCLCPP_INFO(_node.get_logger(), "小码已重新识别");
		}
	}
}

// 新增：发送PX4命令函数实现（参数含默认值）
void PrecisionLand::send_vehicle_command(uint16_t command, 
                                         float param1, 
                                         float param2,
                                         float param3, 
                                         float param4, 
                                         double param5,
                                         double param6, 
                                         float param7)
{
	auto msg = std::make_unique<px4_msgs::msg::VehicleCommand>();
	msg->timestamp = _node.now().nanoseconds() / 1000; // 纳秒转微秒
	msg->param1 = param1;
	msg->param2 = param2;
	msg->param3 = param3;
	msg->param4 = param4;
	msg->param5 = param5;
	msg->param6 = param6;
	msg->param7 = param7;
	msg->command = command;
	msg->target_system = 1;
	msg->target_component = 1;
	msg->source_system = 1;
	msg->source_component = 1;
	msg->confirmation = 0;
	msg->from_external = true;

	_vehicle_command_pub->publish(std::move(msg));
	RCLCPP_INFO(_node.get_logger(), "发送PX4命令: command=%u, param1=%.1f", command, param1);
}

//新增：执行降落命令函数
void PrecisionLand::execute_land()
{
    // 1. 获取当前无人机状态（解锁状态）
    bool armed = false;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        armed = current_vehicle_status_.arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
    }

    // 2. 检查是否已解锁（未解锁则不发送降落命令）
    if (!armed) {
        RCLCPP_WARN(_node.get_logger(), "无人机未解锁，无法发送降落命令");
        return;
    }

    // 3. 发送降落命令（VEHICLE_CMD_NAV_LAND=21，参数与模板一致）
    send_vehicle_command(VEHICLE_CMD_NAV_LAND);
    RCLCPP_INFO(_node.get_logger(), "降落命令已发送,等待PX4执行");
}

//新增：激光高度回调函数
void PrecisionLand::laserHeightCallback(const px4_msgs::msg::DistanceSensor::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(height_mutex_);
    
    // 检查激光高度数据的有效性
    if (msg->type == msg->MAV_DISTANCE_SENSOR_LASER && 
        msg->orientation == msg->ROTATION_DOWNWARD_FACING &&
        msg->current_distance >= msg->min_distance && 
        msg->current_distance <= msg->max_distance) {
        
        // 1. 将有效数据加入滑动缓冲区
        _laser_height_buffer.push_back(msg->current_distance);
        
        // 2. 保持缓冲区大小不超过设定的窗口（例如5帧）
        if (_laser_height_buffer.size() > static_cast<size_t>(_laser_buffer_size)) {
            _laser_height_buffer.pop_front();
        }
        
        // 3. 计算滑动平均值（避免单帧数据跳变）
        double sum = 0.0;
        for (const auto& h : _laser_height_buffer) {
            sum += h;
        }
        _last_laser_height = sum / _laser_height_buffer.size();
        
        // 4. 标记数据有效并打印日志（改用DEBUG级别避免刷屏）
        _laser_data_valid = true;
        // RCLCPP_DEBUG(_node.get_logger(), 
        //              "激光高度更新 [滑动平均]: 原始=%.2f m, 滤波后=%.2f m (缓冲区大小=%ld)",
        //              msg->current_distance, _last_laser_height, _laser_height_buffer.size());
                     
    } else {
        // 无效数据处理：标记无效，清空缓冲区避免残留错误值
        _laser_data_valid = false;
        _laser_height_buffer.clear(); // 清空缓冲区，防止无效数据参与后续计算
        RCLCPP_WARN_THROTTLE(_node.get_logger(), *(_node.get_clock()), 1000, 
                             "激光数据无效 - 类型:%d, 朝向:%d, 距离:%.3f (有效范围:%.3f~%.3f)",
                             msg->type, msg->orientation, msg->current_distance, 
                             msg->min_distance, msg->max_distance);
    }
}


//新增：高度融合函数实现
double PrecisionLand::fuseHeight(double visual_height, double laser_height)
{
	std::lock_guard<std::mutex> lock(height_mutex_);
	// 权重归一化
	double total_weight = _param_visual_weight + _param_laser_weight;
	double visual_w = _param_visual_weight / total_weight;
	double laser_w = _param_laser_weight / total_weight;
	// 数据有效性检查
	bool visual_valid = !std::isnan(visual_height) && visual_height > 0.0;
	bool laser_valid = _laser_data_valid && laser_height >= _laser_min_valid_distance && laser_height <= _laser_max_valid_distance;

	double fused = 0.0;
	if (std::isnan(laser_height)) {
		laser_valid = false; // 激光高度为NaN时视为无效
	}
	
	if (visual_valid && laser_valid) {
		// 双源数据有效，按权重融合
		fused = visual_w * visual_height + laser_w * laser_height;
		//RCLCPP_DEBUG(_node.get_logger(), "高度融合: 视觉=%.2f m, 激光=%.2f m, 融合=%.2f m", visual_height, laser_height, fused);
	} else if (visual_valid) {
		// 仅视觉数据有效
		fused = visual_height;
		//RCLCPP_DEBUG(_node.get_logger(), "仅视觉高度有效: %.2f m", fused);
	} else if (laser_valid) {
		// 仅激光数据有效
		fused = laser_height;
		//RCLCPP_DEBUG(_node.get_logger(), "仅激光高度有效: %.2f m", fused);
	} else {
		// 均无效:使用无人机本地位置高度
		fused = -_vehicle_local_position->positionNed().z();
		//RCLCPP_WARN(_node.get_logger(), "视觉和激光高度数据均无效，使用本地位置高度: %.2f m", fused);
	}
	_fused_height = fused;
	return fused;
}

// Helpers: 统一判断 tag 是否可见/新鲜
bool PrecisionLand::tagSeen(const ArucoTag & tag) const
{
	if (!tag.valid()) return false;
	double age = _node.now().seconds() - tag.timestamp.seconds();
	return age <= _param_target_timeout;
}

bool PrecisionLand::tagFresh(const ArucoTag & tag, double timeout) const
{
	if (!tag.valid()) return false;
	double age = _node.now().seconds() - tag.timestamp.seconds();
	return age <= timeout;
}

PrecisionLand::ArucoTag PrecisionLand::getTagWorld(const ArucoTag& tag)
{
	// ********** 坐标系转换 **********
	// 光学相机坐标（Optical） --> NED
	// 你的代码注释：
	// Optical: X right, Y down, Z away from lens
	// NED: X forward, Y right, Z away from viewer
	// 使用一个变换矩阵 R，把 optical 旋转到 NED
	Eigen::Matrix3d R;
	R << 0, -1, 0,
	     1,  0, 0,
	     0,  0, 1;
	Eigen::Quaterniond quat_NED(R); // 四元数表示的旋转（optical->NED）

	// 读取无人机当前在 NED（本地坐标）的位姿
	auto vehicle_position = Eigen::Vector3d(_vehicle_local_position->positionNed().cast<double>());
	auto vehicle_orientation = Eigen::Quaterniond(_vehicle_attitude->attitude().cast<double>());

	// 形成仿射变换（注意乘法顺序：平移 * 旋转）
	// drone_transform：机体在世界的位置+姿态
	Eigen::Affine3d drone_transform = Eigen::Translation3d(vehicle_position) * vehicle_orientation;

	// camera_transform：相机相对于机体的偏置（通过参数配置）和 optical->NED 旋转
	// 注意：实际应用中相机偏置应由标定给出，参数提供运行时/工程灵活性
	Eigen::Affine3d camera_transform = Eigen::Translation3d(_camera_offset_x, _camera_offset_y, _camera_offset_z) * quat_NED;

	// tag_transform：相机检测到的 tag 在相机坐标系的位置+姿态
	Eigen::Affine3d tag_transform = Eigen::Translation3d(tag.position) * tag.orientation;

	// tag 在 world（NED）下的变换
	Eigen::Affine3d tag_world_transform = drone_transform * camera_transform * tag_transform;

	ArucoTag world_tag = {
		.position = tag_world_transform.translation(),
		.orientation = Eigen::Quaterniond(tag_world_transform.rotation()),
		.timestamp = tag.timestamp,
	};

	return world_tag;
}

void PrecisionLand::onActivate()
{
	// 激活时生成搜索航点并开始搜索
	generateSearchWaypoints();
	_prev_yaw_rate = 0.0f; // 重置上次偏航速率
	_search_started = true;
	_last_debug_print_time = _node.now().seconds(); // 新增：重置打印时间，确保激活后立即打印一次
	switchToState(State::Search);
}

void PrecisionLand::onDeactivate()
{
	// 空（可以在这里做清理）
}

//	新增：获取基于二维码的高度（优先使用二维码数据，无有效数据时回退到飞控高度）

double PrecisionLand::getVisualHeightFromTags()
{
    // 线程安全地读取原始高度
    std::lock_guard<std::mutex> lock(_raw_height_mutex);

    // 标记原始高度是否有效（非 NaN、正数、合理范围）
    bool large_valid = !std::isnan(_large_tag_raw_z) && _large_tag_raw_z > 0.0 && _large_tag_raw_z < 20.0;  // 假设最大有效高度 10 米
    bool small_valid = !std::isnan(_small_tag_raw_z) && _small_tag_raw_z > 0.0 && _small_tag_raw_z < 20.0;

    // 情况1：大码和小码同时有效 → 取较小值
    if (large_valid && small_valid) {
        double selected_height = std::min(_large_tag_raw_z, _small_tag_raw_z);
        //RCLCPP_DEBUG(_node.get_logger(), "大码小码同时有效，取较大高度 (大码: %.2f m, 小码: %.2f m) → 选中: %.2f m",
                     //_large_tag_raw_z, _small_tag_raw_z, selected_height);
        return selected_height;
    }

    // 情况2：仅大码有效
    if (large_valid) {
        //RCLCPP_DEBUG(_node.get_logger(), "仅大码有效，使用原始高度: %.2f m", _large_tag_raw_z);
        return _large_tag_raw_z;
    }

    // 情况3：仅小码有效
    if (small_valid) {
        //RCLCPP_DEBUG(_node.get_logger(), "仅小码有效，使用原始高度: %.2f m", _small_tag_raw_z);
        return _small_tag_raw_z;
    }

    // 情况4：无有效二维码 → 回退到飞控高度
    double fallback_height = -_vehicle_local_position->positionNed().z();
    RCLCPP_WARN_THROTTLE(_node.get_logger(), *(_node.get_clock()), 1000,
                         "无有效二维码高度数据，回退到飞控高度: %.2f m", fallback_height);
    return fallback_height;
}

void PrecisionLand::updateSetpoint(float dt_s)
{
// 获取当前无人机高度（NED坐标系：Z轴向下）
double visual_height = getVisualHeightFromTags();	// 视觉高度
double laser_height = _last_laser_height;                            
double current_height = fuseHeight(visual_height, laser_height);    
// 统一选择当前有效的 tag（小码优先）
ArucoTag current_tag; // 当前依赖的二维码位姿
enum TagType {None, Small, Large};
TagType current_type = None;

bool small_seen = tagSeen(_tag_small);
bool large_seen = tagSeen(_tag_large);
if (small_seen) {
    current_tag = _tag_small;
    current_type = Small;
} else if (large_seen) {
    current_tag = _tag_large;
    current_type = Large;
} else {
    // invalid current_tag: keep default NaN in position
}

// 打印目标丢失/找回日志（仅边沿触发）
bool target_lost = (current_type == None);
if (target_lost && !_target_lost_prev) {
	RCLCPP_INFO(_node.get_logger(), "Target lost (current height: %.2f m)", current_height);
} else if (!target_lost && _target_lost_prev) {
	RCLCPP_INFO(_node.get_logger(), "Target acquired (current height: %.2f m)", current_height);
}
_target_lost_prev = target_lost;    // 更新上次丢失状态

	// //新增：调试信息打印控制
	// double current_time = _node.now().seconds();
	// bool need_print = (kEnableDebugOutput &&
	// 	(current_time - _last_debug_print_time >= 0.5)); // 每0.5秒打印一次

	// // 打印调试信息（0.5秒一次）
	// if (need_print) {
	// 	// 准备调试信息
	// 	std::string current_tag_type = current_height > _height_threshold_switch ? "大码(" + std::to_string(_param_aruco_id_large) + ")" : "小码(" + std::to_string(_param_aruco_id_small) + ")";
	// 	bool is_precise_control = (current_height <= _height_threshold_low);
	// 	bool is_land_complete = (_state == State::Finished) || _land_detected;
		
	// 	// 格式化输出激光高度（处理NaN情况）
	// 	std::string laser_height_str = std::isnan(laser_height) ? "无效" : std::to_string(laser_height);
		
	// 	// 打印调试信息
	// 	RCLCPP_INFO(_node.get_logger(), 
	// 		"\n===== 精准降落调试信息 =====\n"
	// 		"视觉高度：%.2f m\t激光高度：%s m\t融合高度：%.2f m\n"
	// 		"当前使用二维码：%s\n"
	// 		"是否进入精控：%s\n"
	// 		"是否完成降落：%s\n"
	// 		"============================",
	// 		visual_height,
	// 		laser_height_str.c_str(),
	// 		current_height,
	// 		current_tag_type.c_str(),
	// 		is_precise_control ? "是" : "否",
	// 		is_land_complete ? "是" : "否");
		
	// 	_last_debug_print_time = current_time; // 更新最后打印时间
	// }

	// 状态机分支
	switch (_state) {
	case State::Idle: {
		// 空转
		break;
	}

	case State::Search: {
		// Search 中：若检测到任一有效二维码，则进入 Approach
		if (current_type != None) {
			RCLCPP_INFO(_node.get_logger(), "目标已识别,切换到Approach");
			_approach_altitude = _vehicle_local_position->positionNed().z();
			// 进入 Approach 前清除积分等状态
			_vel_x_integral = 0.f; _vel_y_integral = 0.f;
			_small_tag_lost = false;
			switchToState(State::Approach);
			break;
		}

		// 否则按航点搜索
		auto waypoint_position = _search_waypoints[_search_waypoint_index];
		_trajectory_setpoint->updatePosition(waypoint_position); // 位置 setpoint

		// 判断是否到达该航点，若到达则切换到下一个航点（循环）
		if (positionReached(waypoint_position)) {
			_search_waypoint_index++;

			if (_search_waypoint_index >= static_cast<int>(_search_waypoints.size())) {
				_search_waypoint_index = 0;
			}
		}

		break;
	}

// --------------------------- 修改二：在 updateSetpoint 的 Approach 分支改为速度闭环渐近（避免 position setpoint 导致飞控生成高速轨迹） ---------------------------
case State::Approach: {
    // 计算当前水平距离到目标（只 XY）- 保留原代码核心距离计算
    float dx = _vehicle_local_position->positionNed().x() - current_tag.position.x();
    float dy = _vehicle_local_position->positionNed().y() - current_tag.position.y();
    float dist_xy = std::sqrt(dx*dx + dy*dy);
    const float approach_radius = 1.0f; // 原代码的近距切换阈值（可参数化）

	if (current_height > _height_threshold_switch) {
			// 高空：优先大码，其次小码，均不存在返回Search
			// 重新判断有效tag（优先大码）
			if (large_seen) {
				current_tag = _tag_large;
				current_type = Large;
				RCLCPP_DEBUG(_node.get_logger(), "高空模式：使用大码进行控制");
			} else if (small_seen) {
				current_tag = _tag_small;
				current_type = Small;
				RCLCPP_DEBUG(_node.get_logger(), "高空模式：大码丢失，使用小码进行控制");
			} else {
				// 大小码均不存在，返回Search
				RCLCPP_INFO(_node.get_logger(), "Approach(高空): 大小码均丢失，返回Search");
				_small_tag_lost = false;
				_recover_active = false; // 重置回升状态
				switchToState(State::Search);
				return;
			}

			// 高空有效tag存在时，按距离分级控制（远-速度闭环，近-位置闭环）
			if (dist_xy > approach_radius) {
				// 远距：速度闭环逐步接近
				Eigen::Vector2f vel = calculateVelocitySetpointXY(current_tag, dt_s);
				px4_ros2::TrajectorySetpoint setpoint;

				// 获取当前航向（rad），使用 px4_ros2::OdometryAttitude::yaw()
				float current_yaw = 0.0f;
				if (_vehicle_attitude) {
					current_yaw = _vehicle_attitude->yaw();
				}

				float target_yaw = px4_ros2::quaternionToYaw(current_tag.orientation);

				// 使用仓库中可能已有的 _max_yaw_speed（确认单位为 rad/s；若为 deg/s，请换算）
				float max_yaw_rate = _max_yaw_speed; // 若该变量不存在，请替换为合适的常量或成员变量

				float yaw_rate_cmd = computeYawRateToAlign(current_yaw, target_yaw,
														max_yaw_rate,
														_yaw_kp,
														_yaw_deadband,
														_prev_yaw_rate,
														_yaw_alpha);

				_prev_yaw_rate = yaw_rate_cmd;

				setpoint.withVelocity(Eigen::Vector3f(vel.x(), vel.y(), 0.0f))
						.withYawRate(yaw_rate_cmd); // 只发 yaw rate，不发 yaw angle

				_trajectory_setpoint->update(setpoint);
				// RCLCPP_DEBUG(_node.get_logger(), "高空模式：远距速度闭环（dist_xy=%.2f m）", dist_xy);
			// 高空模式-近距位置闭环分支
			}
			else {
				RCLCPP_DEBUG(_node.get_logger(), "高空模式：进入近距位置闭环（dist_xy=%.2f m）", dist_xy);
					auto target_position = Eigen::Vector3f(current_tag.position.x(), current_tag.position.y(), _approach_altitude);
					px4_ros2::TrajectorySetpoint setpoint;

					float current_yaw = 0.0f;
					if (_vehicle_attitude) {
						current_yaw = _vehicle_attitude->yaw();
					}
					float target_yaw = px4_ros2::quaternionToYaw(current_tag.orientation);
					float max_yaw_rate = _max_yaw_speed;
					float yaw_rate_cmd = computeYawRateToAlign(current_yaw, target_yaw,
															max_yaw_rate,
															_yaw_kp,
															_yaw_deadband,
															_prev_yaw_rate,
															_yaw_alpha);
					_prev_yaw_rate = yaw_rate_cmd;

					setpoint.withPosition(target_position)
							.withYawRate(yaw_rate_cmd);
					_trajectory_setpoint->update(setpoint);
				// 打印positionReached的关键参数，看为何不达标
				auto position = _vehicle_local_position->positionNed();
				auto velocity = _vehicle_local_position->velocityNed();
				float delta_pos = (target_position - position).norm();
				float vel_norm = velocity.norm();
				RCLCPP_DEBUG(_node.get_logger(), "目标位置(X:%.2f,Y:%.2f,Z:%.2f) | 当前位置(X:%.2f,Y:%.2f,Z:%.2f) | 位置误差:%.3f m | 速度:%.3f m/s",
					target_position.x(), target_position.y(), target_position.z(),
					position.x(), position.y(), position.z(),
					delta_pos, vel_norm);
				if (positionReached(target_position)) {
					RCLCPP_INFO(_node.get_logger(), "到达目标上方,切换到Descend");
					switchToState(State::Descend);
				}
			}

	} else {
		// 低空：优先小码，大码作为备选，均不存在则回升
		if (current_type == Small) {
			// 小码存在：按距离分级控制（原逻辑保留）
			if (dist_xy > approach_radius) {
				Eigen::Vector2f vel = calculateVelocitySetpointXY(current_tag, dt_s);
				px4_ros2::TrajectorySetpoint setpoint;

				float current_yaw = 0.0f;
				if (_vehicle_attitude) {
					current_yaw = _vehicle_attitude->yaw();
				}
				float target_yaw = px4_ros2::quaternionToYaw(current_tag.orientation);
				float max_yaw_rate = _max_yaw_speed;
				float yaw_rate_cmd = computeYawRateToAlign(current_yaw, target_yaw,
														max_yaw_rate,
														_yaw_kp,
														_yaw_deadband,
														_prev_yaw_rate,
														_yaw_alpha);
				_prev_yaw_rate = yaw_rate_cmd;

				setpoint.withVelocity(Eigen::Vector3f(vel.x(), vel.y(), 0.0f))
						.withYawRate(yaw_rate_cmd);
				_trajectory_setpoint->update(setpoint);
			} else {
				auto target_position = Eigen::Vector3f(current_tag.position.x(), current_tag.position.y(), _approach_altitude);
				px4_ros2::TrajectorySetpoint setpoint;

				float current_yaw = 0.0f;
				if (_vehicle_attitude) {
					current_yaw = _vehicle_attitude->yaw();
				}
				float target_yaw = px4_ros2::quaternionToYaw(current_tag.orientation);
				float max_yaw_rate = _max_yaw_speed;
				float yaw_rate_cmd = computeYawRateToAlign(current_yaw, target_yaw,
														max_yaw_rate,
														_yaw_kp,
														_yaw_deadband,
														_prev_yaw_rate,
														_yaw_alpha);
				_prev_yaw_rate = yaw_rate_cmd;

				setpoint.withPosition(target_position)
						.withYawRate(yaw_rate_cmd);
				_trajectory_setpoint->update(setpoint);
				if (positionReached(target_position)) {
					RCLCPP_INFO(_node.get_logger(), "到达目标上方,切换到Descend");
					switchToState(State::Descend);
				}
			}

		} else if (current_type == Large) {
			// 仅大码存在：复用距离分级控制逻辑（与小码处理一致）
			if (dist_xy > approach_radius) {
				// 远距：速度闭环接近（使用大码位姿计算）
				Eigen::Vector2f vel = calculateVelocitySetpointXY(current_tag, dt_s);
				px4_ros2::TrajectorySetpoint setpoint;

				float current_yaw = 0.0f;
				if (_vehicle_attitude) {
					current_yaw = _vehicle_attitude->yaw();
				}
				float target_yaw = px4_ros2::quaternionToYaw(current_tag.orientation);
				float max_yaw_rate = _max_yaw_speed;
				float yaw_rate_cmd = computeYawRateToAlign(current_yaw, target_yaw,
														max_yaw_rate,
														_yaw_kp,
														_yaw_deadband,
														_prev_yaw_rate,
														_yaw_alpha);
				_prev_yaw_rate = yaw_rate_cmd;

				setpoint.withVelocity(Eigen::Vector3f(vel.x(), vel.y(), 0.0f))
						.withYawRate(yaw_rate_cmd);
				_trajectory_setpoint->update(setpoint);
			} else {
				// 近距：位置闭环到大码上方
				auto target_position = Eigen::Vector3f(current_tag.position.x(), current_tag.position.y(), _approach_altitude);
				px4_ros2::TrajectorySetpoint setpoint;

				float current_yaw = 0.0f;
				if (_vehicle_attitude) {
					current_yaw = _vehicle_attitude->yaw();
				}
				float target_yaw = px4_ros2::quaternionToYaw(current_tag.orientation);
				float max_yaw_rate = _max_yaw_speed;
				float yaw_rate_cmd = computeYawRateToAlign(current_yaw, target_yaw,
														max_yaw_rate,
														_yaw_kp,
														_yaw_deadband,
														_prev_yaw_rate,
														_yaw_alpha);
				_prev_yaw_rate = yaw_rate_cmd;

				setpoint.withPosition(target_position)
						.withYawRate(yaw_rate_cmd);
				_trajectory_setpoint->update(setpoint);
				if (positionReached(target_position)) {
					RCLCPP_INFO(_node.get_logger(), "到达大码上方,切换到Descend");
					switchToState(State::Descend);
				}
			}

		} else {
			// 大小码均不存在：执行原小码丢失回升流程
			if (!_small_tag_lost) {
				_small_tag_lost = true;
				_small_tag_lost_time = _node.now();
				// 初始化回升目标：从当前高度逐步回升
				double current_z = _vehicle_local_position->positionNed().z();
				_recover_target_z = std::max(current_z - _param_recover_step_height, -_height_threshold_recover);
				_recover_active = true;
				RCLCPP_INFO(_node.get_logger(), "大小码均丢失，开始回升寻找（初始回升目标Z: %.2f）", _recover_target_z);
			}

			// 执行回升：保持当前XY位置，上升到目标Z
			_trajectory_setpoint->updatePosition(Eigen::Vector3f(
				_vehicle_local_position->positionNed().x(),
				_vehicle_local_position->positionNed().y(),
				_recover_target_z));

			// 检查是否达到回升目标
			double cur_z = _vehicle_local_position->positionNed().z();
			if (std::abs(cur_z - _recover_target_z) < _param_delta_position) {
				// 回升到位后检查是否找回小码或大码
				if (!tagFresh(_tag_small, _param_small_loss_confirm_time) && !tagFresh(_tag_large, _param_small_loss_confirm_time)) {
					// 仍未找到任何码：继续回升，直到达到分界高度
					double next_target = _recover_target_z - _param_recover_step_height; // NED坐标系：z越小高度越高
					if (next_target <= -_height_threshold_switch) {
						// 已回升到高空分界高度，返回Search
						RCLCPP_INFO(_node.get_logger(), "回升到分界高度仍未找到码，返回Search");
						_recover_active = false;
						_small_tag_lost = false;
						switchToState(State::Search);
						return;
					} else {
						// 继续向更高处回升
						_recover_target_z = next_target;
						RCLCPP_INFO(_node.get_logger(), "仍未找到码，继续回升到Z: %.2f", _recover_target_z);
					}
				} else {
					// 找回任一码：取消回升，返回Approach
					_recover_active = false;
					_small_tag_lost = false;
					RCLCPP_INFO(_node.get_logger(), "回升过程中找回码，返回Approach");
					switchToState(State::Approach);
					return;
				}
			}
		}
	}


    break;
}

	case State::Descend: {

		// 降落中：高度大于低阈值时，持续速度闭环控制，如果current_tag丢失则切换到Approach
            if (current_height > _height_threshold_low) {
				    Eigen::Vector2f vel = calculateVelocitySetpointXY(current_tag, dt_s);
					px4_ros2::TrajectorySetpoint setpoint;

					float current_yaw = 0.0f;
					if (_vehicle_attitude) {
						current_yaw = _vehicle_attitude->yaw();
					}
					float target_yaw = px4_ros2::quaternionToYaw(current_tag.orientation);
					float max_yaw_rate = _max_yaw_speed;
					float yaw_rate_cmd = computeYawRateToAlign(current_yaw, target_yaw,
															max_yaw_rate,
															_yaw_kp,
															_yaw_deadband,
															_prev_yaw_rate,
															_yaw_alpha);
					_prev_yaw_rate = yaw_rate_cmd;

					setpoint.withVelocity(Eigen::Vector3f(vel.x(), vel.y(), _param_descent_vel))
							.withYawRate(yaw_rate_cmd);
					_trajectory_setpoint->update(setpoint);
					RCLCPP_INFO(_node.get_logger(), "Descend(高空) - 发送速度: X=%.3f, Y=%.3f, Z=%.3f (m/s)",
            vel.x(), vel.y(), _param_descent_vel);
            } else if (current_type == None && current_height > _land_mode_switch_height) {
				// 低阈值内丢失目标，返回Approach
				RCLCPP_INFO(_node.get_logger(), "Descend: 目标丢失，返回Approach");
				// 进入 Approach 前清除积分等状态
				_vel_x_integral = 0.f; _vel_y_integral = 0.f;
				_small_tag_lost = false;
				switchToState(State::Approach);
				return;
			}
			else {
        // 最终降落阶段，如果current_tag有效，则继续速度闭环控制，速度减半
			if (current_type != None) {
				Eigen::Vector2f vel = calculateVelocitySetpointXY(current_tag, dt_s);
				px4_ros2::TrajectorySetpoint setpoint;
				float current_yaw = 0.0f;
				if (_vehicle_attitude) {
					current_yaw = _vehicle_attitude->yaw();
				}
				float target_yaw = px4_ros2::quaternionToYaw(current_tag.orientation);
				float max_yaw_rate = _max_yaw_speed;
				float yaw_rate_cmd = computeYawRateToAlign(current_yaw, target_yaw,
														max_yaw_rate,
														_yaw_kp,
														_yaw_deadband,
														_prev_yaw_rate,
														_yaw_alpha);
				_prev_yaw_rate = yaw_rate_cmd;
				setpoint.withVelocity(Eigen::Vector3f(vel.x(), vel.y(), _param_descent_vel * 0.5))
						.withYawRate(yaw_rate_cmd);
				_trajectory_setpoint->update(setpoint);
				RCLCPP_INFO(_node.get_logger(), "Descend(低空) - 发送速度: X=%.3f, Y=%.3f, Z=%.3f (m/s)",
			vel.x(), vel.y(), _param_descent_vel * 0.5);
			} else {
				// current_tag无效，高度低于land_mode_switch_height允许切换到Land模式的最低高度，切换到Land模式
				if (current_height <= _land_mode_switch_height) {
					RCLCPP_INFO(_node.get_logger(), "Descend: 目标丢失且高度低于 %.2f m,切换到Land模式", _land_mode_switch_height);
					execute_land(); // 发送降落命令
					switchToState(State::Finished);
				} else {
					// 否则继续保持当前位置，等待目标恢复
					_trajectory_setpoint->updatePosition(Eigen::Vector3f(
						_vehicle_local_position->positionNed().x(),
						_vehicle_local_position->positionNed().y(),
						_vehicle_local_position->positionNed().z()));
				}
			}
		}
		break;
	}

	case State::Finished: {
		// 通知上层模式完成
		ModeBase::completed(px4_ros2::Result::Success);
		break;
	}
	} // end switch/case
}

// ---------- 修正：实现与头文件中声明一致的函数签名 ----------
// 注意：头文件中声明为:
//     Eigen::Vector2f calculateVelocitySetpointXY(const ArucoTag& current_tag, double dt);
// 因此这里必须实现相同签名，并使用 PrecisionLand::ArucoTag 来避免访问限定带来的编译问题。
Eigen::Vector2f PrecisionLand::calculateVelocitySetpointXY(const PrecisionLand::ArucoTag& current_tag, double dt)
{
    // 防护：确保 dt 合理
    if (dt <= 0.0 || dt > 1.0) {
        dt = 0.02; // 回退到默认 50Hz
    }

    float p_gain = _param_vel_p_gain;	// P component gain
    float i_gain = _param_vel_i_gain;	// I component gain

    // P component: 当前机体位置减去 tag 位置,位置误差： 无人机位置 - 目标位置（NED）
    float delta_pos_x = _vehicle_local_position->positionNed().x() - static_cast<float>(current_tag.position.x());	// X 方向误差
    float delta_pos_y = _vehicle_local_position->positionNed().y() - static_cast<float>(current_tag.position.y());	// Y 方向误差

    // 如果tag位置无效（NaN），则清零积分，直接返回零速度
    if (!current_tag.valid()) {
        _vel_x_integral = 0.f;
        _vel_y_integral = 0.f;
        return Eigen::Vector2f(0.f, 0.f);
    }

    // 计算比例项 （未饱和的原始比例输出）
    float proposed_Xp = delta_pos_x * p_gain;
    float proposed_Yp = delta_pos_y * p_gain;

    // 计算积分限幅（确保积分项单独作用时不会超过最大速度）
    float integral_limit = 0.0f;
    if (std::abs(i_gain) > 1e-9f && _param_max_velocity > 1e-9f) {
        integral_limit = std::abs(_param_max_velocity / i_gain);
    } else {
        integral_limit = std::abs(_param_max_velocity); // 避免除零，使用最大速度作为保底限幅
    }

    // 临时保存当前积分值（用于抗饱和回滚）
    float prev_x_integral = _vel_x_integral;
    float prev_y_integral = _vel_y_integral;

    // 累积积分项（引入dt，使积分与时间间隔相关）
    _vel_x_integral += delta_pos_x * static_cast<float>(dt);
    _vel_y_integral += delta_pos_y * static_cast<float>(dt);

    // 积分限幅
    _vel_x_integral = std::clamp(_vel_x_integral, -integral_limit, integral_limit);
    _vel_y_integral = std::clamp(_vel_y_integral, -integral_limit, integral_limit);

    // 计算完整的PI输出（比例+积分)
    float Xi = _vel_x_integral * i_gain;
    float Yi = _vel_y_integral * i_gain;

    float vx = - (proposed_Xp + Xi);  // 负号：控制方向朝向目标
    float vy = - (proposed_Yp + Yi);

    // 速度限幅（确保输出不超过最大速度）
    float vx_clamped = std::clamp(vx, -_param_max_velocity, _param_max_velocity);
    float vy_clamped = std::clamp(vy, -_param_max_velocity, _param_max_velocity);

    // 检测是否饱和（输出被限幅）
    bool saturated_x = (vx != vx_clamped);
    bool saturated_y = (vy != vy_clamped);

    // 抗积分饱和：如果输出饱和，回滚本次积分（防止积分累积导致的饱和问题）
    if (saturated_x) {
        _vel_x_integral = prev_x_integral;  // 撤销本次积分
    }
    if (saturated_y) {
        _vel_y_integral = prev_y_integral;  // 撤销本次积分
    }

    return Eigen::Vector2f(vx_clamped, vy_clamped);
}

bool PrecisionLand::checkTargetTimeout(const ArucoTag& tag)
{
	// 若 tag 本身无效，认为已丢失
	if (!tag.valid()) {
		return true;
	}

	// 若当前时间与 tag 时间差超过阈值，认为已丢失
	if (_node.now().seconds() - tag.timestamp.seconds() > _param_target_timeout) {
		return true;
	}

	return false;
}

void PrecisionLand::generateSearchWaypoints()
{
	// 生成一个“螺旋”搜索路径（NED）
	double start_x = 0.0;
	double start_y = 0.0;
	double current_z = -_fused_height; // 融合高度转NED（Z向下）
	auto min_z = -1.0; // 目标最低高度（NED 约定）
	double max_radius = 2.0;
	double layer_spacing = 0.5;
	int points_per_layer = 16;
	std::vector<Eigen::Vector3f> waypoints;

	// 计算需要的层数（注意符号/四舍五入）
	int num_layers = (static_cast<int>((min_z - current_z) / layer_spacing) / 2) < 1 ? 1 : (static_cast<int>((
				 min_z - current_z) / layer_spacing) / 2);

	// 为每一层产生外螺旋与内螺旋（并改变高度）
	for (int layer = 0; layer < num_layers; ++layer) {
		std::vector<Eigen::Vector3f> layer_waypoints;

		// 从中心向外螺旋
		double radius = 0.0;
		for (int point = 0; point < points_per_layer + 1; ++point) {
			double angle = 2.0 * M_PI * point / points_per_layer;
			double x = start_x + radius * cos(angle);
			double y = start_y + radius * sin(angle);
			double z = current_z;
			layer_waypoints.push_back(Eigen::Vector3f(x, y, z));
			radius += max_radius / points_per_layer; // 逐步增大半径
		}

		// 将该层的外螺旋加入总航点
		waypoints.insert(waypoints.end(), layer_waypoints.begin(), layer_waypoints.end());

		// 向内螺旋过渡时降低高度
		current_z += layer_spacing;

		// 内螺旋：将顺序反过来，然后设置 z，加入到主航点
		std::reverse(layer_waypoints.begin(), layer_waypoints.end());
		for (auto& waypoint : layer_waypoints) {
			waypoint.z() = current_z;
		}
		waypoints.insert(waypoints.end(), layer_waypoints.begin(), layer_waypoints.end());

		// 为下一层继续降低高度
		current_z += layer_spacing;
	}

	_search_waypoints = waypoints;
}

bool PrecisionLand::positionReached(const Eigen::Vector3f& target) const
{
	// 读取当前位置和速度
	auto position = _vehicle_local_position->positionNed();
	auto velocity = _vehicle_local_position->velocityNed();

	const auto delta_pos = target - position;
	// 到达条件：位置误差小于阈值 && 速度小于阈值
	return (delta_pos.norm() < _param_delta_position) && (velocity.norm() < _param_delta_velocity);
}

std::string PrecisionLand::stateName(State state)
{
	switch (state) {
	case State::Idle:
		return "Idle";
	case State::Search:
		return "Search";
	case State::Approach:
		return "Approach";
	case State::Descend:
		return "Descend";
	case State::Finished:
		return "Finished";
	default:
		return "Unknown";
	}
}

void PrecisionLand::switchToState(State state)
{
	RCLCPP_INFO(_node.get_logger(), "Switching to %s", stateName(state).c_str());
	_state = state;
}

int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);
	// NodeWithMode 会把 PrecisionLand 包装成一个 ROS2 node 并处理 mode 的生命周期（例如周期性调用 updateSetpoint）
	rclcpp::spin(std::make_shared<px4_ros2::NodeWithMode<PrecisionLand>>(kModeName, kEnableDebugOutput));
	rclcpp::shutdown();
	return 0;
}
