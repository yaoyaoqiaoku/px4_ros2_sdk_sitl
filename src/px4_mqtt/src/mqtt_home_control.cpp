#include "mqtt_home_control.hpp"
#include "px4_msgs/msg/home_position_command.hpp"
#include <chrono>
#include <thread>
#include <mutex>

using namespace std::chrono_literals;
using json = nlohmann::json;

namespace px4_mqtt
{

MQTTHomeControl::MQTTHomeControl() : Node("mqtt_home_control")
{
    // 获取参数
    this->declare_parameter<std::string>("uav_name", "uav1");
    this->declare_parameter<std::string>("mqtt_broker", "tcp://localhost:1883");
    this->declare_parameter<std::string>("mqtt_username", "");
    this->declare_parameter<std::string>("mqtt_password", "");
    this->declare_parameter<std::string>("mqtt_home_topic", "uavcontrol/home/");
    
    uav_name_ = this->get_parameter("uav_name").as_string();
    mqtt_broker_ = this->get_parameter("mqtt_broker").as_string();
    mqtt_username_ = this->get_parameter("mqtt_username").as_string();
    mqtt_password_ = this->get_parameter("mqtt_password").as_string();
    std::string topic_prefix = this->get_parameter("mqtt_home_topic").as_string();
    mqtt_home_topic_ = topic_prefix + uav_name_;
    
    // 转换MQTT地址格式
    if (mqtt_broker_.find("mqtt://") == 0) {
        mqtt_broker_.replace(0, 6, "tcp://");
    }

    RCLCPP_INFO(this->get_logger(), "MQTT Home Control starting");
    RCLCPP_INFO(this->get_logger(), "UAV: %s, Broker: %s, Topic: %s", 
                uav_name_.c_str(), mqtt_broker_.c_str(), mqtt_home_topic_.c_str());

    setup_publishers();
    init_mqtt();
    
    // MQTT重连定时器
    mqtt_reconnect_timer_ = this->create_wall_timer(
        5s, std::bind(&MQTTHomeControl::try_reconnect_mqtt, this));
}

void MQTTHomeControl::setup_publishers()
{
    // 创建Home位置命令发布者
    home_command_pub_ = this->create_publisher<px4_msgs::msg::HomePositionCommand>(
        "/px4_home/command", rclcpp::QoS(10));
    
    RCLCPP_INFO(this->get_logger(), "Created ROS publisher on /px4_home/command");
}

void MQTTHomeControl::connection_lost(const std::string& /*cause*/)
{
    std::lock_guard<std::mutex> lock(mqtt_mutex_);
    mqtt_connected_ = false;
    reconnect_attempts_ = 0;
    RCLCPP_WARN(this->get_logger(), "MQTT connection lost");
}

void MQTTHomeControl::message_arrived(mqtt::const_message_ptr msg)
{
    try {
        std::string topic = msg->get_topic();
        std::string payload = msg->to_string();
        
        RCLCPP_DEBUG(this->get_logger(), "Received MQTT home command: %s", payload.c_str());
        
        process_mqtt_command(topic, payload);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error processing MQTT message: %s", e.what());
    }
}

void MQTTHomeControl::delivery_complete(mqtt::delivery_token_ptr /*token*/)
{
    // 可选：处理消息发送确认
}

void MQTTHomeControl::init_mqtt()
{
    try {
        // 生成客户端ID
        std::string client_id = "mqtt_home_" + uav_name_ + "_" +
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
                    mqtt_home_topic_.c_str());
    } catch (const mqtt::exception& e) {
        std::lock_guard<std::mutex> lock(mqtt_mutex_);
        mqtt_connected_ = false;
        RCLCPP_ERROR(this->get_logger(), "MQTT connection failed: %s", e.what());
    }
}

void MQTTHomeControl::subscribe_to_mqtt_topics()
{
    try {
        std::lock_guard<std::mutex> lock(mqtt_mutex_);
        if (mqtt_client_ && mqtt_client_->is_connected()) {
            mqtt_client_->subscribe(mqtt_home_topic_, 0);
            RCLCPP_INFO(this->get_logger(), "Subscribed to MQTT topic: %s", mqtt_home_topic_.c_str());
        }
    } catch (const mqtt::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to subscribe to MQTT topic: %s", e.what());
    }
}

void MQTTHomeControl::try_reconnect_mqtt()
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

bool MQTTHomeControl::is_mqtt_connected()
{
    std::lock_guard<std::mutex> lock(mqtt_mutex_);
    return mqtt_connected_ && mqtt_client_ && mqtt_client_->is_connected();
}

void MQTTHomeControl::process_mqtt_command(const std::string& topic, const std::string& payload)
{
    try {
        json cmd = json::parse(payload);
        
        // 检查是否为Home位置命令
        if (cmd.contains("home_position")) {
            handle_home_command(cmd);
        } else if (cmd.contains("command")) {
            // 也可以支持简单的命令格式
            std::string command = cmd["command"].get<std::string>();
            if (command == "set_home") {
                handle_home_command(cmd);
            }
        }
    } catch (const nlohmann::json::parse_error& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to parse JSON command: %s", e.what());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error processing command: %s", e.what());
    }
}

void MQTTHomeControl::handle_home_command(const json& cmd)
{
    bool use_current = false;
    double lat = 0.0;
    double lon = 0.0;
    float alt = 0.0f;
    bool update = true;
    
    // 解析Home位置参数
    if (cmd.contains("use_current")) {
        use_current = cmd["use_current"].get<bool>();
    }
    
    if (!use_current) {
        if (cmd.contains("lat")) {
            lat = cmd["lat"].get<double>();
        }
        if (cmd.contains("lon")) {
            lon = cmd["lon"].get<double>();
        }
        if (cmd.contains("alt")) {
            alt = cmd["alt"].get<float>();
        }
    }
    
    if (cmd.contains("update")) {
        update = cmd["update"].get<bool>();
    }
    
    RCLCPP_INFO(this->get_logger(), "Processing home command: use_current=%s, lat=%.6f, lon=%.6f, alt=%.1f", 
                use_current ? "true" : "false", lat, lon, alt);
    
    publish_home_command(use_current, lat, lon, alt, update);
}

void MQTTHomeControl::publish_home_command(bool use_current, double lat, double lon, float alt, bool update)
{
    auto msg = px4_msgs::msg::HomePositionCommand();
    
    msg.use_current = use_current;
    msg.lat = lat;
    msg.lon = lon;
    msg.alt = alt;
    msg.update = update;
    
    // 发布到ROS话题
    home_command_pub_->publish(msg);
    
    if (use_current) {
        RCLCPP_INFO(this->get_logger(), "Published home command: use current position");
    } else {
        RCLCPP_INFO(this->get_logger(), "Published home command: lat=%.6f, lon=%.6f, alt=%.1f", 
                    lat, lon, alt);
    }
}

} // namespace px4_mqtt

// 主函数
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<px4_mqtt::MQTTHomeControl>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}