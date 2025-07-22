#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "dynamixel_sdk_custom_interfaces/msg/set_position.hpp"
#include <map>
#include <string>
#include <algorithm>
#include <chrono>
#include <memory>
#include <vector>
#include <cmath>
#include <cstdlib>
#include "SCServo.h"

using namespace std::chrono_literals;

class ServoMapperNode : public rclcpp::Node {
public:
    ServoMapperNode() : Node("servo_mapper_node") {
        // Declare parameters
        this->declare_parameter<std::string>("port", "/dev/ttyACM0");
        this->declare_parameter<int>("baudrate", 1000000);
        
        joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 50,
            std::bind(&ServoMapperNode::joint_callback, this, std::placeholders::_1));
        
        // Publisher für Dynamixel (funktioniert bereits)
        set_position_publisher_ = this->create_publisher<dynamixel_sdk_custom_interfaces::msg::SetPosition>("set_position", 10);
        
        // Timer für kontrollierte Update-Rate
        timer_ = this->create_wall_timer(36ms, std::bind(&ServoMapperNode::timer_callback, this));
        
        RCLCPP_INFO(this->get_logger(), "Servo Mapper Node started!");

        // SCServo initialisieren
        std::string port = this->get_parameter("port").as_string();
        int baudrate = this->get_parameter("baudrate").as_int();
        
        if(!sm_st.begin(baudrate, port.c_str())){
            RCLCPP_ERROR(this->get_logger(), "Failed to init SCServo at %s!", port.c_str());
            rclcpp::shutdown();
            return;
        }
        RCLCPP_INFO(this->get_logger(), "SCServo initialized at %s", port.c_str());

        // Initialisierung
        initialize_servos();
    }

    ~ServoMapperNode() {
        sm_st.end();
    }

private:
    // Dynamixel Servos (funktionieren bereits, keine Logs)
    std::map<std::string, int> dynamixel_map = {
        {"thumb_opposition", 1},
        {"thumb_rotation", 2},
        {"index_mcp_adduction", 3},
        {"middle_mcp_adduction", 4},
        {"ring_mcp_adduction", 5}
    };
    
    // SCServos (diese debuggen wir)
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

    // Kalibrierung
    std::map<std::string, double> offset_map = {
        {"thumb_opposition", -0.127},
        {"thumb_rotation", +0.365},
        {"index_mcp_adduction", 0.0},
        {"middle_mcp_adduction", 0.0},
        {"ring_mcp_adduction", 0.0},
        {"index_mcp_flexion", 0.0},
        {"index_pip", 0.0},
        {"middle_mcp_flexion", 0.0},
        {"middle_pip", 0.0},
        {"ring_mcp_flexion", 0.0},
        {"ring_pip", 0.0},
        {"thumb_mcp_flexion", 0.0},
        {"thumb_pip", 0.0}
    };
    
    std::map<std::string, double> multiplier_map = {
        {"thumb_opposition", -1.0},
        {"thumb_rotation", -1.0},
        {"index_mcp_adduction", -1.0},
        {"middle_mcp_adduction", -1.0},
        {"ring_mcp_adduction", -1.0},
        {"index_mcp_flexion", 1.0},
        {"index_pip", 1.0},
        {"middle_mcp_flexion", 1.0},
        {"middle_pip", 1.0},
        {"ring_mcp_flexion", 1.0},
        {"ring_pip", 1.0},
        {"thumb_mcp_flexion", 1.0},
        {"thumb_pip", 1.0}
    };

    // Gelenk-Limits
    std::map<std::string, std::pair<double, double>> joint_limits = {
        {"thumb_opposition", {-0.35, 1.28}},
        {"thumb_rotation", {-1.57, 1.57}},
        {"index_mcp_adduction", {-0.5, 0.5}},
        {"middle_mcp_adduction", {-0.5, 0.5}},
        {"ring_mcp_adduction", {-0.5, 0.5}},
        {"index_mcp_flexion", {-1.57, 1.57}},
        {"index_pip", {-1.57, 1.57}},
        {"middle_mcp_flexion", {-1.57, 1.57}},
        {"middle_pip", {-1.57, 1.57}},
        {"ring_mcp_flexion", {-1.57, 1.57}},
        {"ring_pip", {-1.57, 1.57}},
        {"thumb_mcp_flexion", {-1.57, 1.57}},
        {"thumb_pip", {-1.57, 1.57}}
    };

    // Aktuelle Joint-Werte
    std::map<std::string, double> current_joint_values_;
    std::map<std::string, int> last_sent_positions_;

    // Hardware objects
    SMS_STS sm_st;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::Publisher<dynamixel_sdk_custom_interfaces::msg::SetPosition>::SharedPtr set_position_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Umrechnungen
    int rad_to_dynamixel(double rad) {
        int pos = static_cast<int>( (rad + 1.57) * (4095.0 / (2 * 1.57)) );
        return std::clamp(pos, 0, 4095);
    }
    
    int rad_to_scservo(double rad) {
        // -1.57 bis +1.57 rad → 0 bis 4095
        int pos = static_cast<int>( (rad + 1.57) * (4095.0 / (2 * 1.57)) );
        return std::clamp(pos, 0, 4095);
    }

    void initialize_servos() {
        // Dynamixel (still arbeiten)
        for (const auto& [joint, servo_id] : dynamixel_map) {
            double value = 0.0;
            int pos = rad_to_dynamixel(value);
            current_joint_values_[joint] = value;
            last_sent_positions_[joint] = pos;
            auto msg = dynamixel_sdk_custom_interfaces::msg::SetPosition();
            msg.id = servo_id;
            msg.position = pos;
            set_position_publisher_->publish(msg);
        }
        
        // SCServos (loggen)
        std::vector<u8> scservo_ids;
        std::vector<s16> scservo_positions;
        std::vector<u16> scservo_speeds;
        std::vector<u8> scservo_accs;
        
        for (const auto& [joint, servo_id] : scservo_map) {
            double value = 0.0;
            int pos = rad_to_scservo(value);
            current_joint_values_[joint] = value;
            last_sent_positions_[joint] = pos;
            
            scservo_ids.push_back(servo_id);
            scservo_positions.push_back(pos);
            scservo_speeds.push_back(2400);
            scservo_accs.push_back(50);
            
            RCLCPP_INFO(this->get_logger(), "SCServo %s (ID %d) -> %d", joint.c_str(), servo_id, pos);
        }
        
        if (!scservo_ids.empty()) {
            sm_st.SyncWritePosEx(scservo_ids.data(), scservo_ids.size(), 
                                scservo_positions.data(), scservo_speeds.data(), scservo_accs.data());
            RCLCPP_INFO(this->get_logger(), "SCServos initialized");
        }
    }

    void joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        // Verarbeite alle Joints, aber logge nur SCServo-Änderungen
        for (size_t i = 0; i < msg->name.size(); ++i) {
            const std::string& joint = msg->name[i];
            double value = msg->position[i];
            double original_value = value;
            
            // Nur bekannte Joints verarbeiten
            if (!dynamixel_map.count(joint) && !scservo_map.count(joint)) continue;
            
            // Limits anwenden
            if (joint_limits.count(joint)) {
                value = std::max(joint_limits[joint].first, std::min(value, joint_limits[joint].second));
            }
            
            // Offset/Multiplier anwenden
            if (offset_map.count(joint) && multiplier_map.count(joint)) {
                value = value * multiplier_map[joint] + offset_map[joint];
            }
            
            // NUR SCServo Änderungen loggen
            if (scservo_map.count(joint)) {
                bool has_changed = !current_joint_values_.count(joint) || 
                                  std::abs(current_joint_values_[joint] - value) > 0.01;
                
                if (has_changed) {
                    RCLCPP_INFO(this->get_logger(), "SCServo Joint '%s': %.3f -> %.3f (ID %d)", 
                               joint.c_str(), original_value, value, scservo_map[joint]);
                }
            }
            
            current_joint_values_[joint] = value;
        }
    }
    
    void timer_callback() {
        if (current_joint_values_.empty()) return;
        
        // SCServo Updates sammeln
        std::vector<u8> scservo_ids;
        std::vector<s16> scservo_positions;
        std::vector<u16> scservo_speeds;
        std::vector<u8> scservo_accs;
        
        int scservo_changes = 0;
        
        for (const auto& [joint, value] : current_joint_values_) {
            
            if (dynamixel_map.count(joint)) {
                // Dynamixel (still arbeiten, keine Logs)
                int servo_id = dynamixel_map[joint];
                int new_pos = rad_to_dynamixel(value);
                
                auto msg = dynamixel_sdk_custom_interfaces::msg::SetPosition();
                msg.id = servo_id;
                msg.position = new_pos;
                set_position_publisher_->publish(msg);
                last_sent_positions_[joint] = new_pos;
                
            } else if (scservo_map.count(joint)) {
                // SCServo (debuggen)
                int servo_id = scservo_map[joint];
                int new_pos = rad_to_scservo(value);
                int last_pos = last_sent_positions_[joint];
                
                if (new_pos != last_pos) {
                    RCLCPP_INFO(this->get_logger(), "SCServo %s (ID %d): %d->%d", 
                               joint.c_str(), servo_id, last_pos, new_pos);
                    scservo_changes++;
                }
                
                scservo_ids.push_back(servo_id);
                scservo_positions.push_back(new_pos);
                scservo_speeds.push_back(2400);
                scservo_accs.push_back(50);
                last_sent_positions_[joint] = new_pos;
            }
        }
        
        // SCServo Batch senden
        if (!scservo_ids.empty()) {
            sm_st.SyncWritePosEx(scservo_ids.data(), scservo_ids.size(), 
                                scservo_positions.data(), scservo_speeds.data(), scservo_accs.data());
            
            if (scservo_changes > 0) {
                RCLCPP_INFO(this->get_logger(), "Sent %d SCServo changes", scservo_changes);
            }
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ServoMapperNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}