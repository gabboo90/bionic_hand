#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <map>
#include <string>
#include <algorithm>
#include "dynamixel_sdk_custom_interfaces/msg/set_position.hpp"

class ServoMapperNode : public rclcpp::Node {
public:
    ServoMapperNode() : Node("servo_mapper_node") {
        joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&ServoMapperNode::joint_callback, this, std::placeholders::_1));
        dynamixel_pub_ = this->create_publisher<dynamixel_sdk_custom_interfaces::msg::SetPosition>("/set_position", 10);
        RCLCPP_INFO(this->get_logger(), "ServoMapperNode gestartet!");
    }

private:
    std::map<std::string, int> dynamixel_map = {
        {"thumb_opposition", 1},
        {"thumb_rotation", 2},
        {"index_mcp_adduction", 3},
        {"middle_mcp_adduction", 4},
        {"ring_mcp_adduction", 5}
    };
    std::map<std::string, int> scservo_map = {
        {"index_mcp_flexion", 1},
        {"index_pip", 2},
        {"middle_mcp_flexion", 3},
        {"middle_pip", 4},
        {"ring_mcp_flexion", 5},
        {"ring_pip", 6},
        {"thumb_mcp_flexion", 7},
        {"thumb_pip", 8}
    };

    int rad_to_dynamixel(double rad) {
        int pos = static_cast<int>( (rad + 1.57) * (4095.0 / (2 * 1.57)) );
        return std::clamp(pos, 0, 4095);
    }
    int rad_to_scservo(double rad) {
        int pos = static_cast<int>( (rad + 1.57) * (1023.0 / (2 * 1.57)) );
        return std::clamp(pos, 0, 1023);
    }

    void joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        for (size_t i = 0; i < msg->name.size(); ++i) {
            const std::string& joint = msg->name[i];
            double value = msg->position[i];
            if (joint == "thumb_opposition") {
                int pos = rad_to_dynamixel(value);
                auto setpos = dynamixel_sdk_custom_interfaces::msg::SetPosition();
                setpos.id = 1; // Dynamixel-ID für thumb_opposition
                setpos.position = pos;
                dynamixel_pub_->publish(setpos);
                RCLCPP_INFO(this->get_logger(), "Published thumb_opposition to Dynamixel ID 1: %d", pos);
            } else if (dynamixel_map.count(joint)) {
                int servo_id = dynamixel_map[joint];
                int pos = rad_to_dynamixel(value);
                // TODO: Sende an Dynamixel-Servo servo_id, Position pos (weitere Achsen)
                RCLCPP_INFO(this->get_logger(), "Dynamixel %s (ID %d): %d", joint.c_str(), servo_id, pos);
            } else if (scservo_map.count(joint)) {
                int servo_id = scservo_map[joint];
                int pos = rad_to_scservo(value);
                // TODO: Sende an SCServo-Servo servo_id, Position pos
                RCLCPP_INFO(this->get_logger(), "SCServo %s (ID %d): %d", joint.c_str(), servo_id, pos);
            }
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::Publisher<dynamixel_sdk_custom_interfaces::msg::SetPosition>::SharedPtr dynamixel_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ServoMapperNode>());
    rclcpp::shutdown();
    return 0;
} 