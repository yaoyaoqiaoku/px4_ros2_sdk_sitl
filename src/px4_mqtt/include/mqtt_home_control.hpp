#ifndef PX4_MQTT_MQTT_HOME_CONTROL_HPP_
#define PX4_MQTT_MQTT_HOME_CONTROL_HPP_

#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/home_position_command.hpp"
#include <mqtt/client.h>
#include <nlohmann/json.hpp>
#include <string>
#include <mutex>

namespace px4_mqtt
{

class MQTTHomeControl : public rclcpp::Node, public virtual mqtt::callback
{
public:
    MQTTHomeControl();
    
private:
    void setup_publishers();
    void init_mqtt();
    void subscribe_to_mqtt_topics();
    void try_reconnect_mqtt();
    bool is_mqtt_connected();
    
    void connection_lost(const std::string& cause) override;
    void message_arrived(mqtt::const_message_ptr msg) override;
    void delivery_complete(mqtt::delivery_token_ptr token) override;
    
    void process_mqtt_command(const std::string& topic, const std::string& payload);
    void handle_home_command(const nlohmann::json& cmd);
    void publish_home_command(bool use_current, double lat, double lon, float alt, bool update = true);
    
    // ROS发布者
    rclcpp::Publisher<px4_msgs::msg::HomePositionCommand>::SharedPtr home_command_pub_;
    
    // MQTT相关
    std::unique_ptr<mqtt::client> mqtt_client_;
    std::mutex mqtt_mutex_;
    bool mqtt_connected_ = false;
    int reconnect_attempts_ = 0;
    const int max_reconnect_attempts_ = 10;
    
    // 配置参数
    std::string uav_name_;
    std::string mqtt_broker_;
    std::string mqtt_username_;
    std::string mqtt_password_;
    std::string mqtt_home_topic_;
    
    // 定时器
    rclcpp::TimerBase::SharedPtr mqtt_reconnect_timer_;
};

} // namespace px4_mqtt

#endif // PX4_MQTT_MQTT_HOME_CONTROL_HPP_