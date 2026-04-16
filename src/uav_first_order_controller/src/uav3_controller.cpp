#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <map>
#include <array>
#include <cmath>
#include <atomic>
#include <algorithm>
using namespace std::chrono_literals;

// ============================================================================
// 配置区域：无人机ID、邻居关系和初始位置（新增统一上升参数）
// ============================================================================
const int UAV_ID = 3;  // 需要为每架无人机单独设置：1, 2, 3, 4, 5, 6
const int TARGET_SYSTEM = UAV_ID + 1; // 多机场景：px4_N对应target_system=N+1（单机可设为1）

// 邻居关系定义（无向图，ID从1开始）
const std::map<int, std::vector<int>> NEIGHBORS = {
    {1, {2, 3}},        // uav1 连接 uav2, uav3
    {2, {1, 3, 4, 5}},  // uav2 连接 uav1, uav3, uav4, uav5
    {3, {1, 2, 5, 6}},  // uav3 连接 uav1, uav2, uav5, uav6
    {4, {2, 5}},        // uav4 连接 uav2, uav5
    {5, {2, 3, 4, 6}},  // uav5 连接 uav2, uav3, uav4, uav6 (原点无人机)
    {6, {3, 5}}         // uav6 连接 uav3, uav5
};

// 初始相对位置（在世界坐标系中，相对于虚拟领航者）
const std::map<int, std::array<float, 3>> INIT_REL_POS = {
    {1, {0.0f, 20.0f, -40.0f}},   // uav1: x=0, y=20, z=-40
    {2, {5.0f, 10.0f, -40.0f}},   // uav2: x=5, y=10, z=-40
    {3, {-5.0f, 10.0f, -40.0f}},  // uav3: x=-5, y=10, z=-40
    {4, {10.0f, 0.0f, -40.0f}},   // uav4: x=10, y=0, z=-40
    {5, {0.0f, 0.0f, -40.0f}},    // uav5: x=0, y=0, z=-40 (原点)
    {6, {-10.0f, 0.0f, -40.0f}}   // uav6: x=-10, y=0, z=-40
};

// 新增：统一上升控制参数（所有无人机共用）
const float UNIFORM_CLIMB_SPEED = 0.5f;  // 统一爬升速度 (m/s)，建议0.3~0.8
const float CLIMB_HORIZONTAL_POS_LOCK = true; // 爬升阶段锁定水平初始位置
const float POS_CMD_FILTER_ALPHA = 0.8f; // 位置指令滤波系数（0~1，越大越平滑）
const float POS_ERROR_DEADZONE = 0.1f;   // 位置误差死区 (m)，小于该值不调整
const float MAX_CMD_CHANGE = 0.2f;       // 位置指令最大突变幅度 (m/100ms)

// ============================================================================
// 一阶一致性控制器类（优化版）
// ============================================================================
class FirstOrderController : public rclcpp::Node
{
public:
    FirstOrderController() : Node("first_order_controller_" + std::to_string(UAV_ID))
    {
        RCLCPP_INFO(this->get_logger(), "初始化无人机 %d 的控制器 (target_system=%d) - 优化版位置控制", UAV_ID, TARGET_SYSTEM);

        // --------------------------------------------------------------------
        // 1. 创建发布者（保持官方QoS）
        // --------------------------------------------------------------------
        offboard_control_mode_publisher_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
            "/px4_" + std::to_string(UAV_ID) + "/fmu/in/offboard_control_mode", 10);
        trajectory_setpoint_publisher_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            "/px4_" + std::to_string(UAV_ID) + "/fmu/in/trajectory_setpoint", 10);
        vehicle_command_publisher_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
            "/px4_" + std::to_string(UAV_ID) + "/fmu/in/vehicle_command", 10);

        // 邻居位置广播发布者
        rclcpp::QoS broadcast_qos(1);
        broadcast_qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
        broadcast_qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
        broadcast_qos.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);
        pos_broadcast_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
            "/uav_positions", broadcast_qos);

        // --------------------------------------------------------------------
        // 2. 创建订阅者
        // --------------------------------------------------------------------
        // 订阅自身位置
        own_pos_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/px4_" + std::to_string(UAV_ID) + "/fmu/out/vehicle_odometry", broadcast_qos,
            [this](const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
                own_pos_ = {msg->position[0], msg->position[1], msg->position[2]};
                own_pos_valid_ = true;
                // 记录首次有效位置（用于爬升阶段水平锁定）
                if (!first_pos_recorded_) {
                    first_own_pos_ = own_pos_;
                    first_pos_recorded_ = true;
                    RCLCPP_INFO(this->get_logger(), "记录初始位置: [%.2f, %.2f, %.2f]",
                               first_own_pos_[0], first_own_pos_[1], first_own_pos_[2]);
                }
                static int count = 0;
                if (count++ % 50 == 0) {
                    RCLCPP_DEBUG(this->get_logger(), "自身位置: [%.2f, %.2f, %.2f]", 
                                own_pos_[0], own_pos_[1], own_pos_[2]);
                }
            });

        // 订阅邻居位置
        neighbor_pos_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/uav_positions", broadcast_qos,
            [this](const geometry_msgs::msg::PointStamped::SharedPtr msg) {
                int sender_id = std::stoi(msg->header.frame_id);
                if (sender_id != UAV_ID) {
                    auto neighbors_it = NEIGHBORS.find(UAV_ID);
                    if (neighbors_it != NEIGHBORS.end()) {
                        auto& neighbor_list = neighbors_it->second;
                        if (std::find(neighbor_list.begin(), neighbor_list.end(), sender_id) != neighbor_list.end()) {
                            neighbor_pos_[sender_id] = {msg->point.x, msg->point.y, msg->point.z};
                            neighbor_last_update_[sender_id] = this->now();
                        }
                    }
                }
            });

        // --------------------------------------------------------------------
        // 3. 初始化参数（新增滤波相关）
        // --------------------------------------------------------------------
        offboard_setpoint_counter_ = 0;
        timestamp_.store(this->get_clock()->now().nanoseconds() / 1000);
        start_time_ = this->now();
        height_reached_ = false;
        first_pos_recorded_ = false;
        
        // 控制器核心参数（微调增益，提升稳定性）
        k_ = 0.3f;          // 降低增益，减少震荡（原0.5）
        omega_ = 0.05f;     // 虚拟领航者圆周运动角速度
        radius_ = 50.0f;    // 虚拟领航者圆周运动半径
        target_z_ = INIT_REL_POS.at(UAV_ID)[2]; 
        height_tolerance_ = 0.3f; // 收紧高度容忍度（原0.5）

        // 新增：滤波缓存
        filtered_pos_cmd_ = INIT_REL_POS.at(UAV_ID); // 初始滤波指令

        // --------------------------------------------------------------------
        // 4. 定时器（100ms=10Hz，与PX4匹配）
        // --------------------------------------------------------------------
        auto timer_callback = [this]() -> void {
            // 核心逻辑1：模式切换+解锁
            if (offboard_setpoint_counter_ == 10) {
                this->publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0);
                this->arm();
                RCLCPP_INFO(this->get_logger(), "发送Offboard模式切换+解锁命令（计数器=10）");
            }

            // 核心逻辑2：发布控制模式
            publish_offboard_control_mode();
            
            // 分阶段控制逻辑（优化版）
            if (offboard_setpoint_counter_ < 11) {
                publish_hover_setpoint(); // 初始悬停指令
            } else {
                if (!height_reached_ && own_pos_valid_) {
                    // 检查高度是否到达（收紧判断）
                    if (std::abs(own_pos_[2] - target_z_) < height_tolerance_) {
                        height_reached_ = true;
                        RCLCPP_INFO(this->get_logger(), "已到达目标高度 %.2f m，开始水平编队位置控制", target_z_);
                    } else {
                        publish_uniform_climb_setpoint(); // 统一速度爬升（替代原hover）
                    }
                } else {
                    publish_consistency_setpoint_optimized(); // 优化版一致性控制
                }
            }

            // 计数器控制
            if (offboard_setpoint_counter_ < 11) {
                offboard_setpoint_counter_++;
            }

            // 广播自身位置
            broadcastOwnPosition();
        };
        timer_ = this->create_wall_timer(100ms, timer_callback);

        RCLCPP_INFO(this->get_logger(), "控制器初始化完成，UAV_ID=%d，目标高度=%.2f m，统一爬升速度=%.2f m/s", 
                   UAV_ID, target_z_, UNIFORM_CLIMB_SPEED);
    }

    void arm();
    void disarm();

private:
    // ------------------------------------------------------------------------
    // 成员变量（新增滤波/爬升相关）
    // ------------------------------------------------------------------------
    rclcpp::TimerBase::SharedPtr timer_;

    // PX4控制发布者
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;

    // 邻居通信相关
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pos_broadcast_pub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr own_pos_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr neighbor_pos_sub_;

    std::atomic<uint64_t> timestamp_;
    uint64_t offboard_setpoint_counter_;

    // 控制器状态
    std::array<float, 3> own_pos_ = {0.0f, 0.0f, 0.0f};
    bool own_pos_valid_ = false;
    std::array<float, 3> first_own_pos_ = {0.0f, 0.0f, 0.0f}; // 首次有效位置
    bool first_pos_recorded_ = false;
    std::map<int, std::array<float, 3>> neighbor_pos_;
    std::map<int, rclcpp::Time> neighbor_last_update_;
    rclcpp::Time start_time_;
    
    // z轴控制相关
    bool height_reached_;          
    float target_z_;               
    float height_tolerance_;       

    // 控制器参数
    float k_;          
    float omega_;      
    float radius_;     

    // 新增：滤波缓存
    std::array<float, 3> filtered_pos_cmd_;

    // ------------------------------------------------------------------------
    // 核心函数（优化版）
    // ------------------------------------------------------------------------
    void publish_offboard_control_mode()
    {
        px4_msgs::msg::OffboardControlMode msg{};
        msg.position = true;    
        msg.velocity = false;   
        msg.acceleration = false;
        msg.attitude = false;
        msg.body_rate = false;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        offboard_control_mode_publisher_->publish(msg);
    }

    /**
     * @brief 发布统一速度爬升指令（核心修改：替换原hover_setpoint）
     */
    void publish_uniform_climb_setpoint()
    {
        px4_msgs::msg::TrajectorySetpoint msg{};
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        
        std::array<float, 3> pos_cmd = {0.0f, 0.0f, 0.0f};

        if (own_pos_valid_) {
            // 水平位置：锁定初始位置（确保爬升整齐）
            if (CLIMB_HORIZONTAL_POS_LOCK && first_pos_recorded_) {
                pos_cmd[0] = first_own_pos_[0];
                pos_cmd[1] = first_own_pos_[1];
            } else {
                pos_cmd[0] = own_pos_[0];
                pos_cmd[1] = own_pos_[1];
            }

            // 垂直位置：统一速度爬升（基于时间计算，而非步长）
            float z_desired = own_pos_[2] + UNIFORM_CLIMB_SPEED * 0.1f; // 100ms定时器，所以×0.1
            // 限制最大高度不超过目标值
            pos_cmd[2] = std::min(z_desired, target_z_);

            RCLCPP_DEBUG(this->get_logger(), "统一爬升中：当前高度=%.2f m, 目标=%.2f m, 指令=%.2f m (速度=%.2f m/s)",
                        own_pos_[2], target_z_, pos_cmd[2], UNIFORM_CLIMB_SPEED);
        } else {
            pos_cmd = INIT_REL_POS.at(UAV_ID);
        }

        // 发布指令
        msg.position = pos_cmd;
        msg.velocity = {NAN, NAN, NAN};
        msg.yaw = NAN; 
        trajectory_setpoint_publisher_->publish(msg);
    }

    /**
     * @brief 发布优化版一阶一致性位置控制指令
     */
    void publish_consistency_setpoint_optimized()
    {
        px4_msgs::msg::TrajectorySetpoint msg{};
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        std::array<float, 3> pos_cmd = own_pos_;

        if (own_pos_valid_) {
            checkNeighborDataFreshness();

            // 5号机：跟随虚拟领航者
            if (UAV_ID == 5) {
                pos_cmd = getLeaderPosition();
                pos_cmd[2] = target_z_;
            } else {
                // 其他无人机：优化版一致性控制
                auto leader_pos = getLeaderPosition();
                auto d_i_t = getDesiredRelativePos(UAV_ID);
                std::array<float, 3> base_pos = {
                    leader_pos[0] + d_i_t[0],
                    leader_pos[1] + d_i_t[1],
                    target_z_
                };

                // 优化1：计算一致性误差（归一化邻居数量）
                float error_x = 0.0f, error_y = 0.0f;
                int valid_neighbors = 0;

                auto neighbors_it = NEIGHBORS.find(UAV_ID);
                if (neighbors_it != NEIGHBORS.end()) {
                    for (int neighbor_id : neighbors_it->second) {
                        // 优化2：邻居数据失效时，使用期望位置兜底
                        std::array<float, 3> neighbor_pos;
                        if (neighbor_pos_.find(neighbor_id) != neighbor_pos_.end()) {
                            neighbor_pos = neighbor_pos_[neighbor_id];
                            valid_neighbors++;
                        } else {
                            // 邻居数据过期，使用其期望位置
                            auto d_j_t = getDesiredRelativePos(neighbor_id);
                            neighbor_pos = {
                                leader_pos[0] + d_j_t[0],
                                leader_pos[1] + d_j_t[1],
                                target_z_
                            };
                        }

                        auto d_j_t = getDesiredRelativePos(neighbor_id);
                        auto neighbor_base_pos = std::array<float, 3>{
                            leader_pos[0] + d_j_t[0],
                            leader_pos[1] + d_j_t[1],
                            target_z_
                        };

                        // 计算误差
                        float dx = (own_pos_[0] - neighbor_pos[0]) - (base_pos[0] - neighbor_base_pos[0]);
                        float dy = (own_pos_[1] - neighbor_pos[1]) - (base_pos[1] - neighbor_base_pos[1]);

                        // 优化3：死区处理，小误差不调整
                        dx = (std::abs(dx) > POS_ERROR_DEADZONE) ? dx : 0.0f;
                        dy = (std::abs(dy) > POS_ERROR_DEADZONE) ? dy : 0.0f;

                        error_x += dx;
                        error_y += dy;
                    }
                }

                // 归一化误差（避免邻居多的无人机调整幅度过大）
                if (valid_neighbors > 0) {
                    error_x /= valid_neighbors;
                    error_y /= valid_neighbors;
                }

                // 计算补偿后的指令
                pos_cmd[0] = base_pos[0] - k_ * error_x;
                pos_cmd[1] = base_pos[1] - k_ * error_y;
                pos_cmd[2] = target_z_;

                // 优化4：限制指令突变幅度（避免抖动）
                pos_cmd[0] = limitCmdChange(pos_cmd[0], filtered_pos_cmd_[0]);
                pos_cmd[1] = limitCmdChange(pos_cmd[1], filtered_pos_cmd_[1]);
                
                // 优化5：低通滤波平滑指令
                filtered_pos_cmd_[0] = POS_CMD_FILTER_ALPHA * filtered_pos_cmd_[0] + (1 - POS_CMD_FILTER_ALPHA) * pos_cmd[0];
                filtered_pos_cmd_[1] = POS_CMD_FILTER_ALPHA * filtered_pos_cmd_[1] + (1 - POS_CMD_FILTER_ALPHA) * pos_cmd[1];
                filtered_pos_cmd_[2] = target_z_;

                // 使用滤波后的指令
                pos_cmd = filtered_pos_cmd_;
            }
        }

        // 发布指令
        msg.position = pos_cmd;
        msg.velocity = {NAN, NAN, NAN};
        msg.yaw = NAN; 
        trajectory_setpoint_publisher_->publish(msg);

        // 日志输出（优化：打印误差）
        static int log_count = 0;
        if (log_count++ % 10 == 0) {
            float error_x = pos_cmd[0] - own_pos_[0];
            float error_y = pos_cmd[1] - own_pos_[1];
            RCLCPP_INFO(this->get_logger(), "编队控制指令: [%.2f, %.2f, %.2f] m | 当前位置: [%.2f, %.2f, %.2f] m | 误差: [%.2f, %.2f] m", 
                       pos_cmd[0], pos_cmd[1], pos_cmd[2], own_pos_[0], own_pos_[1], own_pos_[2], error_x, error_y);
        }
    }

    /**
     * @brief 限制指令突变幅度（避免无人机剧烈调整）
     */
    float limitCmdChange(float new_cmd, float last_cmd)
    {
        float delta = new_cmd - last_cmd;
        if (delta > MAX_CMD_CHANGE) {
            return last_cmd + MAX_CMD_CHANGE;
        } else if (delta < -MAX_CMD_CHANGE) {
            return last_cmd - MAX_CMD_CHANGE;
        } else {
            return new_cmd;
        }
    }

    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0)
    {
        px4_msgs::msg::VehicleCommand msg{};
        msg.param1 = param1;
        msg.param2 = param2;
        msg.command = command;
        msg.target_system = TARGET_SYSTEM;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        vehicle_command_publisher_->publish(msg);
    }

    // ------------------------------------------------------------------------
    // 辅助函数（保持不变）
    // ------------------------------------------------------------------------
    std::array<float, 3> getLeaderPosition()
    {
        double t = (this->now() - start_time_).seconds();
        return {
            radius_ * std::cos(omega_ * t),
            radius_ * std::sin(omega_ * t),
            target_z_
        };
    }

    std::array<float, 3> getDesiredRelativePos(int uav_id)
    {
        double t = (this->now() - start_time_).seconds();
        float cos_theta = std::cos(omega_ * t);
        float sin_theta = std::sin(omega_ * t);
        auto d0 = INIT_REL_POS.at(uav_id);
        return {
            cos_theta * d0[0] - sin_theta * d0[1],
            sin_theta * d0[0] + cos_theta * d0[1],
            d0[2]
        };
    }

    void broadcastOwnPosition()
    {
        if (!own_pos_valid_) {
            return;
        }

        geometry_msgs::msg::PointStamped msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = std::to_string(UAV_ID);
        msg.point.x = own_pos_[0];
        msg.point.y = own_pos_[1];
        msg.point.z = own_pos_[2];
        pos_broadcast_pub_->publish(msg);
    }

    void checkNeighborDataFreshness()
    {
        auto now = this->now();
        std::vector<int> expired_neighbors;

        for (const auto& [id, last_update] : neighbor_last_update_) {
            auto age = (now - last_update).seconds();
            if (age > 1.0) {
                expired_neighbors.push_back(id);
            }
        }

        for (int id : expired_neighbors) {
            neighbor_pos_.erase(id);
            neighbor_last_update_.erase(id);
        }
    }

    // 保留原hover函数（初始化阶段使用）
    void publish_hover_setpoint()
    {
        px4_msgs::msg::TrajectorySetpoint msg{};
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        std::array<float, 3> pos_cmd = INIT_REL_POS.at(UAV_ID);
        msg.position = pos_cmd;
        msg.velocity = {NAN, NAN, NAN};
        msg.yaw = NAN;
        trajectory_setpoint_publisher_->publish(msg);
    }
};

// ----------------------------------------------------------------------------
// arm/disarm函数实现
// ----------------------------------------------------------------------------
void FirstOrderController::arm()
{
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
    RCLCPP_INFO(this->get_logger(), "解锁命令已发送");
}

void FirstOrderController::disarm()
{
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
    RCLCPP_INFO(this->get_logger(), "上锁命令已发送");
}

// ============================================================================
// 主函数
// ============================================================================
int main(int argc, char *argv[])
{
    std::cout << "Starting optimized first order controller node for UAV " << UAV_ID << "..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FirstOrderController>());
    rclcpp::shutdown();
    return 0;
}
