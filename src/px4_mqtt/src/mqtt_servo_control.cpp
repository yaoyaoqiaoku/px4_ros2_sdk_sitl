#include "mqtt_servo_control.hpp"
#include "px4_msgs/msg/servo_command.hpp"
#include <chrono>
#include <thread>
#include <mutex>
#include <array>

using namespace std::chrono_literals;
using json = nlohmann::json;

namespace px4_mqtt
{

MQTTServoControl::MQTTServoControl() : Node("mqtt_servo_control")
{
    // 获取参数
    this->declare_parameter<std::string>("uav_name", "uav1");
    this->declare_parameter<std::string>("mqtt_broker", "tcp://localhost:1883");
    this->declare_parameter<std::string>("mqtt_username", "");
    this->declare_parameter<std::string>("mqtt_password", "");
    this->declare_parameter<std::string>("mqtt_servo_topic", "uavcontrol/servo/");
    
    uav_name_ = this->get_parameter("uav_name").as_string();
    mqtt_broker_ = this->get_parameter("mqtt_broker").as_string();
    mqtt_username_ = this->get_parameter("mqtt_username").as_string();
    mqtt_password_ = this->get_parameter("mqtt_password").as_string();
    std::string topic_prefix = this->get_parameter("mqtt_servo_topic").as_string();
    mqtt_servo_topic_ = topic_prefix + uav_name_;
    
    // 转换MQTT地址格式
    if (mqtt_broker_.find("mqtt://") == 0) {
        mqtt_broker_.replace(0, 6, "tcp://");
    }

    RCLCPP_INFO(this->get_logger(), "MQTT Servo Control starting");
    RCLCPP_INFO(this->get_logger(), "UAV: %s, Broker: %s, Topic: %s", 
                uav_name_.c_str(), mqtt_broker_.c_str(), mqtt_servo_topic_.c_str());

    setup_publishers();
    init_mqtt();
    
    // MQTT重连定时器
    mqtt_reconnect_timer_ = this->create_wall_timer(
        5s, std::bind(&MQTTServoControl::try_reconnect_mqtt, this));
}

void MQTTServoControl::setup_publishers()
{
    // 创建舵机命令发布者
    servo_command_pub_ = this->create_publisher<px4_msgs::msg::ServoCommand>(
        "/px4_servo/command", rclcpp::QoS(10));
    
    RCLCPP_INFO(this->get_logger(), "Created ROS publisher on /px4_servo/command");
}

void MQTTServoControl::connection_lost(const std::string& /*cause*/)
{
    std::lock_guard<std::mutex> lock(mqtt_mutex_);
    mqtt_connected_ = false;
    reconnect_attempts_ = 0;
    RCLCPP_WARN(this->get_logger(), "MQTT connection lost");
}

void MQTTServoControl::message_arrived(mqtt::const_message_ptr msg)
{
    try {
        std::string topic = msg->get_topic();
        std::string payload = msg->to_string();
        
        RCLCPP_DEBUG(this->get_logger(), "Received MQTT servo command: %s", payload.c_str());
        
        process_mqtt_command(topic, payload);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error processing MQTT message: %s", e.what());
    }
}

void MQTTServoControl::delivery_complete(mqtt::delivery_token_ptr /*token*/)
{
    // 可选：处理消息发送确认
}

void MQTTServoControl::init_mqtt()
{
    try {
        // 生成客户端ID
        std::string client_id = "mqtt_servo_" + uav_name_ + "_" +
                                std::to_string(this->now().nanoseconds()) + "_" +
                                std::to_string(getpid());
        
        mqtt_client_ = std::make_unique<mqtt::client>(mqtt_broker_, client_id);
        mqtt_client_->set_callback(*this);
        
        mqtt::connect_options opts;
        opts.set_clean_session(true);
        opts.set_automatic_reconnect(true);
        opts.set_keep_alive_interval(60);
        opts.set_connect_timeout(10);
        
        if (!mqtt_username_.empty() && !mqtt_password_.empty()) {
            opts.set_user_name(mqtt_username_);
            opts.set_password(mqtt_password_);
        }
        
        mqtt_client_->connect(opts);
        {
            std::lock_guard<std::mutex> lock(mqtt_mutex_);
            mqtt_connected_ = true;
            reconnect_attempts_ = 0;
        }
        
        subscribe_to_mqtt_topics();
        
        RCLCPP_INFO(this->get_logger(), "Connected to MQTT server and subscribed to topic: %s", 
                    mqtt_servo_topic_.c_str());
    } catch (const mqtt::exception& e) {
        std::lock_guard<std::mutex> lock(mqtt_mutex_);
        mqtt_connected_ = false;
        RCLCPP_ERROR(this->get_logger(), "MQTT connection failed: %s", e.what());
    }
}

void MQTTServoControl::subscribe_to_mqtt_topics()
{
    try {
        std::lock_guard<std::mutex> lock(mqtt_mutex_);
        if (mqtt_client_ && mqtt_client_->is_connected()) {
            mqtt_client_->subscribe(mqtt_servo_topic_, 0);
            RCLCPP_INFO(this->get_logger(), "Subscribed to MQTT topic: %s", mqtt_servo_topic_.c_str());
        }
    } catch (const mqtt::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to subscribe to MQTT topic: %s", e.what());
    }
}

void MQTTServoControl::try_reconnect_mqtt()
{
    std::lock_guard<std::mutex> lock(mqtt_mutex_);
    
    if (mqtt_connected_) {
        return;
    }
    
    if (reconnect_attempts_ >= max_reconnect_attempts_) {
        return;
    }
    
    reconnect_attempts_++;
    
    try {
        if (mqtt_client_) {
            mqtt::connect_options opts;
            opts.set_clean_session(true);
            opts.set_automatic_reconnect(true);
            opts.set_keep_alive_interval(60);
            opts.set_connect_timeout(10);
            
            if (!mqtt_username_.empty() && !mqtt_password_.empty()) {
                opts.set_user_name(mqtt_username_);
                opts.set_password(mqtt_password_);
            }
            
            mqtt_client_->connect(opts);
            mqtt_connected_ = true;
            reconnect_attempts_ = 0;
            
            subscribe_to_mqtt_topics();
            
            RCLCPP_INFO(this->get_logger(), "MQTT reconnected");
        }
    } catch (const mqtt::exception& /*e*/) {
        mqtt_connected_ = false;
    }
}

bool MQTTServoControl::is_mqtt_connected()
{
    std::lock_guard<std::mutex> lock(mqtt_mutex_);
    return mqtt_connected_ && mqtt_client_ && mqtt_client_->is_connected();
}

void MQTTServoControl::process_mqtt_command(const std::string& topic, const std::string& payload)
{
    try {
        json cmd = json::parse(payload);
        
        // 检查是否为舵机命令
        if (cmd.contains("actuators")) {
            handle_servo_command(cmd);
        } else if (cmd.contains("command")) {
            // 也可以支持简单的命令格式
            std::string command = cmd["command"].get<std::string>();
            if (command == "servo") {
                handle_servo_command(cmd);
            }
        }
    } catch (const nlohmann::json::parse_error& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to parse JSON command: %s", e.what());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error processing command: %s", e.what());
    }
}

void MQTTServoControl::handle_servo_command(const json& cmd)
{
    std::array<float, 6> actuators = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    uint8_t index = 0;
    float frequency = 10.0f;
    bool update = true;
    
    // 解析舵机值
    if (cmd["actuators"].is_array()) {
        auto actuator_array = cmd["actuators"];
        for (size_t i = 0; i < 6 && i < actuator_array.size(); ++i) {
            actuators[i] = actuator_array[i].get<float>();
        }
    }
    
    // 解析其他参数
    if (cmd.contains("actuator_index")) {
        index = cmd["actuator_index"].get<uint8_t>();
    }
    
    if (cmd.contains("send_frequency")) {
        frequency = cmd["send_frequency"].get<float>();
    }
    
    if (cmd.contains("update")) {
        update = cmd["update"].get<bool>();
    }
    
    RCLCPP_INFO(this->get_logger(), "Processing servo command: Index=%d, Freq=%.1fHz", index, frequency);
    
    publish_servo_command(actuators, index, frequency, update);
}

void MQTTServoControl::publish_servo_command(const std::array<float, 6>& actuators, uint8_t index, float frequency, bool update)
{
    auto msg = px4_msgs::msg::ServoCommand();
    
    // 设置舵机值（ServoCommand 中的 actuators 为固定大小数组）
    for (size_t i = 0; i < actuators.size(); ++i) {
        msg.actuators[i] = actuators[i];
    }
    
    msg.actuator_index = index;
    msg.send_frequency = frequency;
    msg.update = update;
    
    // 发布到ROS话题
    servo_command_pub_->publish(msg);
    
    RCLCPP_INFO(this->get_logger(), 
                "Published servo command - Actuators: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]", 
                actuators[0], actuators[1], actuators[2],
                actuators[3], actuators[4], actuators[5]);
}

} // namespace px4_mqtt

// 主函数
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<px4_mqtt::MQTTServoControl>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}