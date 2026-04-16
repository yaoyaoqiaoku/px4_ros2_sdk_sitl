#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node_interfaces/node_topics_interface.hpp>
#include <rclcpp/node_interfaces/node_clock_interface.hpp>
#include <stdint.h>
#include <chrono>
#include <iostream>
#include <cmath>
#include <vector>
#include <memory>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

// 封装单架无人机的Offboard控制逻辑（对齐官方示例核心流程，保留多机封装）
class UAVOffboardAgent
{
public:
    /**
     * @brief 无人机构造函数（对齐官方示例，传递节点核心接口）
     * @param node_topics 节点话题接口（创建发布者）
     * @param node_clock 节点时钟接口（获取时间戳）
     * @param node_name 节点名称（日志输出）
     * @param uav_topic_prefix 无人机话题前缀（/px4_1/、/px4_2/等）
     * @param target_system 无人机target_system编号（对应PX4实例）
     * @param follow_offset 跟随偏移量 [x,y,z]
     */
    UAVOffboardAgent(rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics,
                     rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock,
                     const std::string& node_name,
                     const std::string& uav_topic_prefix, 
                     uint8_t target_system,
                     const std::vector<float>& follow_offset)
        : node_topics_(node_topics),
          node_clock_(node_clock),
          node_name_(node_name),
          uav_topic_prefix_(uav_topic_prefix), 
          target_system_(target_system), 
          follow_offset_(follow_offset),
          offboard_setpoint_counter_(0) // 对齐官方：初始化计数器为0
    {
        // 拼接话题（对齐官方话题格式，适配多机前缀）
        std::string offboard_mode_topic = uav_topic_prefix_ + "fmu/in/offboard_control_mode";
        std::string trajectory_topic = uav_topic_prefix_ + "fmu/in/trajectory_setpoint";
        std::string vehicle_cmd_topic = uav_topic_prefix_ + "fmu/in/vehicle_command";

        // 创建发布者（对齐官方，队列大小10）
        offboard_control_mode_publisher_ = rclcpp::create_publisher<OffboardControlMode>(
            node_topics_, offboard_mode_topic, 10);
        trajectory_setpoint_publisher_ = rclcpp::create_publisher<TrajectorySetpoint>(
            node_topics_, trajectory_topic, 10);
        vehicle_command_publisher_ = rclcpp::create_publisher<VehicleCommand>(
            node_topics_, vehicle_cmd_topic, 10);

        // 初始化位置（对齐官方，悬停在5m高度）
        current_position_[0] = 0.0f;
        current_position_[1] = 0.0f;
        current_position_[2] = -5.0f;
    }

    // 解锁
    void arm() 
    { 
        publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
        RCLCPP_INFO(rclcpp::get_logger(node_name_), "UAV%u: Arm command sent", target_system_);
    }

    // 上锁
    void disarm() 
    { 
        publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0f);
        RCLCPP_INFO(rclcpp::get_logger(node_name_), "UAV%u: Disarm command sent", target_system_);
    }

    // 切换到Offboard模式
    void switch_to_offboard()
    {
        publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);
        RCLCPP_INFO(rclcpp::get_logger(node_name_), "UAV%u: Switch to Offboard mode command sent", target_system_);
    }

    // 发布Offboard控制模式
    void publish_offboard_control_mode()
    {
        OffboardControlMode msg{};
        msg.position = true;  
        msg.velocity = false;
        msg.acceleration = false;
        msg.attitude = false;
        msg.body_rate = false;
        // 对齐官方：时间戳获取方式
        msg.timestamp = node_clock_->get_clock()->now().nanoseconds() / 1000;
        offboard_control_mode_publisher_->publish(msg);
    }

    // 发布轨迹指令（保留多机跟随逻辑）
    void publish_trajectory_setpoint(double t, const float* leader_position)
    {
        TrajectorySetpoint msg{};
        const double radius = 5.0;    // 圆周半径5m
        const double period = 20.0;   // 周期20s
        double omega = 2.0 * M_PI / period; // 角速度

        if (leader_position == nullptr) {
            // 领航机：圆周运动（保留原有业务逻辑，对齐官方位置格式）
            msg.position[0] = static_cast<float>(radius * std::cos(omega * t));
            msg.position[1] = static_cast<float>(radius * std::sin(omega * t));
            msg.position[2] = -5.0f; 
            msg.yaw = -3.14f; // yaw=180度（-PI）

            // 更新自身位置
            current_position_[0] = msg.position[0];
            current_position_[1] = msg.position[1];
            current_position_[2] = msg.position[2];
        } else {
            // 跟随机：跟随偏移（保留原有业务逻辑，对齐官方位置格式）
            msg.position[0] = leader_position[0] + follow_offset_[0];
            msg.position[1] = leader_position[1] + follow_offset_[1];
            msg.position[2] = leader_position[2] + follow_offset_[2];
            msg.yaw = -3.14f; // 对齐官方：统一yaw角度
        }

        // 时间戳获取方式
        msg.timestamp = node_clock_->get_clock()->now().nanoseconds() / 1000;
        trajectory_setpoint_publisher_->publish(msg);

        // 打印位置日志（简化格式，与官方一致）
        RCLCPP_INFO(rclcpp::get_logger(node_name_), "UAV%u: Position - x: %f, y: %f, z: %f", 
                    target_system_, msg.position[0], msg.position[1], msg.position[2]);
    }

    // 对齐官方：获取计数器（供多机节点控制切换时机）
    uint64_t get_offboard_counter() const { return offboard_setpoint_counter_; }

    // 对齐官方：递增计数器（仅前10组指令递增）
    void increment_counter()
    {
        if (offboard_setpoint_counter_ < 11) {
            offboard_setpoint_counter_++;
        }
    }

    // 获取当前无人机位置（供跟随机使用）
    const float* get_current_position() const { return current_position_; }

    // 获取无人机target_system编号
    uint8_t get_target_system() const { return target_system_; }

private:
    // 节点核心接口（多机封装所需）
    rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics_;
    rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock_;
    std::string node_name_;

    // 无人机属性
    std::string uav_topic_prefix_;
    uint8_t target_system_;
    std::vector<float> follow_offset_;
    float current_position_[3];

    // 发布者（与官方示例一一对应）
    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;

    // offboard指令计数器
    uint64_t offboard_setpoint_counter_;

    // 发布车辆指令（严格遵循官方参数格式）
    void publish_vehicle_command(uint16_t command, float param1, float param2 = 0.0f)
    {
        VehicleCommand msg{};
        msg.param1 = param1;
        msg.param2 = param2;
        msg.command = command;
        msg.target_system = target_system_; // 多机适配：对应每架机的target_system
        msg.target_component = 1; // 默认组件编号1
        msg.source_system = 1;    // 默认源系统编号1
        msg.source_component = 1; // 默认源组件编号1
        msg.from_external = true; // 外部指令标记
        msg.timestamp = node_clock_->get_clock()->now().nanoseconds() / 1000;
        
        vehicle_command_publisher_->publish(msg);
    }
};

// 多机控制节点（统一管理所有无人机）
class MultiOffboardControl : public rclcpp::Node
{
public:
    MultiOffboardControl() : Node("multi_offboard_control")
    {
        time_start_ = this->get_clock()->now();

        // -------------------------- 多机配置（后续扩展仅需修改这里）--------------------------
        add_uav("/px4_1/", 2, {0.0f, 0.0f, 0.0f}); // 领航机
        add_uav("/px4_2/", 3, {2.0f, 2.0f, 0.0f});  // 跟随机1
        add_uav("/px4_3/", 4, {2.0f, -2.0f, 0.0f}); // 跟随机2
        add_uav("/px4_4/", 5, {-2.0f, 2.0f, 0.0f}); // 跟随机3
        add_uav("/px4_5/", 6, {-2.0f, -2.0f, 0.0f});// 跟随机4
        add_uav("/px4_6/", 7, {0.0f, 4.0f, 0.0f});  // 跟随机5
        // ----------------------------------------------------------------------------------------------

        // 100ms定时器（10Hz，满足Offboard指令频率要求）
        auto timer_callback = [this]() -> void {
            // 获取领航机位置
            const float* leader_position = uav_agents_[0]->get_current_position();
            auto now = this->get_clock()->now();
            double t = (now - time_start_).seconds();

            // 遍历所有无人机，执行官方核心流程
            for (auto& uav : uav_agents_) {
                // 计数器达到10时，切换Offboard模式+解锁（仅执行一次）
                if (uav->get_offboard_counter() == 10) {
                    uav->switch_to_offboard();
                    uav->arm();
                }

                // 持续配对发布OffboardControlMode和TrajectorySetpoint
                uav->publish_offboard_control_mode();
                uav->publish_trajectory_setpoint(t, leader_position);

                // 计数器递增（仅前10组指令）
                uav->increment_counter();
            }
        };
        timer_ = this->create_wall_timer(100ms, timer_callback);
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<std::shared_ptr<UAVOffboardAgent>> uav_agents_;
    rclcpp::Time time_start_;

    // 多机添加方法
    void add_uav(const std::string& topic_prefix, uint8_t target_system, const std::vector<float>& follow_offset)
    {
        auto uav = std::make_shared<UAVOffboardAgent>(
            this->get_node_topics_interface(),
            this->get_node_clock_interface(),
            this->get_name(),
            topic_prefix,
            target_system,
            follow_offset
        );
        uav_agents_.push_back(uav);
    }
};

int main(int argc, char *argv[])
{
    std::cout << "Starting multi offboard control node..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MultiOffboardControl>());
    rclcpp::shutdown();
    return 0;
}
