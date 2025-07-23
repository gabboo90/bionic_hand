#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <map>
#include <string>
#include <algorithm>
#include <chrono>
#include <memory>
#include <vector>
#include <cmath>
#include <cstdlib>
#include "SCServo.h"
#include "dynamixel_sdk/group_sync_write.h"
#include "dynamixel_sdk/port_handler.h"
#include "dynamixel_sdk/packet_handler.h"
#include <csignal>

using namespace std::chrono_literals;

dynamixel::PortHandler *global_dxl_portHandler = nullptr;

void signal_handler(int signal) {
    if (global_dxl_portHandler) {
        global_dxl_portHandler->closePort();
    }
    rclcpp::shutdown();
    std::exit(0);
}

class ServoMapperNode : public rclcpp::Node {
public:
    ServoMapperNode() : Node("servo_mapper_node") {
        // Neue Parameter für getrennte Ports/Baudraten
        this->declare_parameter<std::string>("scservo_port", "/dev/scservo");
        this->declare_parameter<int>("scservo_baudrate", 1000000);
        this->declare_parameter<std::string>("dynamixel_port", "/dev/dynamixel");
        this->declare_parameter<int>("dynamixel_baudrate", 4000000);
        
        joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 50,
            std::bind(&ServoMapperNode::joint_callback, this, std::placeholders::_1));
        
        // Timer für kontrollierte Update-Rate
        timer_ = this->create_wall_timer(20ms, std::bind(&ServoMapperNode::timer_callback, this));
        
        RCLCPP_INFO(this->get_logger(), "Servo Mapper Node started!");

        // Ports/Baudraten aus Parametern lesen
        std::string scservo_port = this->get_parameter("scservo_port").as_string();
        int scservo_baudrate = this->get_parameter("scservo_baudrate").as_int();
        std::string dynamixel_port = this->get_parameter("dynamixel_port").as_string();
        int dynamixel_baudrate = this->get_parameter("dynamixel_baudrate").as_int();

        // SCServo initialisieren
        if(!sm_st.begin(scservo_baudrate, scservo_port.c_str())){
            RCLCPP_ERROR(this->get_logger(), "Failed to init SCServo at %s!", scservo_port.c_str());
            rclcpp::shutdown();
            return;
        }
        RCLCPP_INFO(this->get_logger(), "SCServo initialized at %s", scservo_port.c_str());

        // DynamixelSDK initialisieren
        dxl_portHandler_ = dynamixel::PortHandler::getPortHandler(dynamixel_port.c_str());
        dxl_packetHandler_ = dynamixel::PacketHandler::getPacketHandler(2.0);

        if (!dxl_portHandler_->openPort()) {
            RCLCPP_ERROR(this->get_logger(), "Dynamixel: Port konnte nicht geöffnet werden (%s)!", dynamixel_port.c_str());
            rclcpp::shutdown();
            return;
        }
        if (!dxl_portHandler_->setBaudRate(dynamixel_baudrate)) {
            RCLCPP_ERROR(this->get_logger(), "Dynamixel: Baudrate konnte nicht gesetzt werden (%d)!", dynamixel_baudrate);
            rclcpp::shutdown();
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Dynamixel initialized at %s", dynamixel_port.c_str());

        // GroupSyncWrite für Goal Position (Adresse 116, 4 Byte)
        dxl_groupSyncWrite_ = std::make_unique<dynamixel::GroupSyncWrite>(dxl_portHandler_, dxl_packetHandler_, 116, 4);

        // Dynamixel-Servos initialisieren (Torque Enable)
        initialize_dynamixel_servos();

        // Initialisierung
        initialize_servos();
    }

    ~ServoMapperNode() {
        sm_st.end();
        if (dxl_portHandler_) {
            dxl_portHandler_->closePort();
        }
    }

    dynamixel::PortHandler* getPortHandler() {
        return dxl_portHandler_;
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
        {"thumb_opposition", -0.3},
        {"thumb_rotation", +0.755},
        {"index_mcp_adduction", + 0.1},
        {"middle_mcp_adduction", + 0.4},
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
        {"thumb_opposition", {-0.3, +1.1}},
        {"thumb_rotation", {-1.57, +1.57}},
        {"index_mcp_adduction", {-0.450, +0.450}},
        {"middle_mcp_adduction", {-0.450, +0.450}},
        {"ring_mcp_adduction", {-0.450, +0.450}},
        {"index_mcp_flexion",   {-1.57, +1.57}},
        {"index_pip", {-1.57, +1.57}},
        {"middle_mcp_flexion", {-1.57, +1.57}},
        {"middle_pip", {-1.57, +1.57}},
        {"ring_mcp_flexion", {-1.57, +1.57}},
        {"ring_pip", {-1.57, +1.57}},
        {"thumb_mcp_flexion", {-1.57, +1.57}},
        {"thumb_pip", {-1.57, +1.57}}
    };
    // Aktuelle Joint-Werte
    std::map<std::string, double> current_joint_values_;
    std::map<std::string, int> last_sent_positions_;

    // Hardware objects
    SMS_STS sm_st;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // DynamixelSDK Objekte
    dynamixel::PortHandler *dxl_portHandler_;
    dynamixel::PacketHandler *dxl_packetHandler_;
    std::unique_ptr<dynamixel::GroupSyncWrite> dxl_groupSyncWrite_;

    // Umrechnungen
    int rad_to_dynamixel(double rad) {
        // -π ... +π rad → 0 ... 4095 (voller 360° Bereich)
        int pos = static_cast<int>(((rad + M_PI) / (2.0 * M_PI)) * 4095.0);
        return std::clamp(pos, 0, 4095);
    }
    
    int rad_to_scservo(double rad) {
        // -1.57 bis +1.57 rad → 0 bis 4095
        int pos = static_cast<int>( (rad + 1.57) * (4095.0 / (2 * 1.57)) );
        return std::clamp(pos, 0, 4095);
    }

    double dynamixel_to_rad(int pos) {
        // 0 ... 4095 → -π ... +π (voller 360° Bereich)
        return ((static_cast<double>(pos) / 4095.0) * (2.0 * M_PI)) - M_PI;
    }

    void initialize_dynamixel_servos() {
        // Torque Enable: Adresse 64, Wert 1
        const uint16_t torque_addr = 64;
        const uint8_t torque_enable = 1;
        for (const auto& [joint, servo_id] : dynamixel_map) {
            // Reboot-Befehl senden
            int reboot_result = dxl_packetHandler_->reboot(dxl_portHandler_, servo_id);
            if (reboot_result == COMM_SUCCESS) {
                RCLCPP_INFO(this->get_logger(), "Dynamixel %s (ID %d): Reboot erfolgreich.", joint.c_str(), servo_id);
            } else {
                RCLCPP_ERROR(this->get_logger(), "Dynamixel %s (ID %d): Reboot fehlgeschlagen! Fehlercode: %d", joint.c_str(), servo_id, reboot_result);
            }
            rclcpp::sleep_for(1s); // 1 Sekunde warten

            int comm_result = dxl_packetHandler_->write1ByteTxRx(dxl_portHandler_, servo_id, torque_addr, torque_enable);
            if (comm_result == COMM_SUCCESS) {
                //RCLCPP_INFO(this->get_logger(), "Dynamixel %s (ID %d): Torque aktiviert.", joint.c_str(), servo_id);
            } else {
                RCLCPP_ERROR(this->get_logger(), "Dynamixel %s (ID %d): Torque-Enable fehlgeschlagen! Fehlercode: %d", joint.c_str(), servo_id, comm_result);
            }
            rclcpp::sleep_for(100ms);
        }
        RCLCPP_INFO(this->get_logger(), "Alle Dynamixel-Servos initialisiert (Reboot + Torque Enable).");
    }

    void initialize_servos() {
        // Dynamixel (keine Logs)
        for (const auto& [joint, servo_id] : dynamixel_map) {
            double value = 0.0;
            int pos = rad_to_dynamixel(value);
            current_joint_values_[joint] = value;
            last_sent_positions_[joint] = pos;
        }
        // SCServos (nur Initialisierung loggen)
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
            scservo_accs.push_back(200);
            //RCLCPP_INFO(this->get_logger(), "SCServo %s (ID %d) -> %d", joint.c_str(), servo_id, pos);
        }
        if (!scservo_ids.empty()) {
            sm_st.SyncWritePosEx(scservo_ids.data(), scservo_ids.size(), 
                                scservo_positions.data(), scservo_speeds.data(), scservo_accs.data());
            RCLCPP_INFO(this->get_logger(), "SCServos initialized");
        }
    }

    void joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        for (size_t i = 0; i < msg->name.size(); ++i) {
            const std::string& joint = msg->name[i];
            double value = msg->position[i];
            //RCLCPP_INFO(this->get_logger(), "Empfangen: %s = %.3f", joint.c_str(), value);
            double original_value = value;
            if (!dynamixel_map.count(joint) && !scservo_map.count(joint)) continue;
            // *** LIMITS WERDEN HIER ANGEWENDET ***
            if (joint_limits.count(joint)) {
                value = std::max(joint_limits[joint].first, std::min(value, joint_limits[joint].second));
            }
            // *** ENDE LIMITS ***
            if (offset_map.count(joint) && multiplier_map.count(joint)) {
                value = value * multiplier_map[joint] + offset_map[joint];
            }
            //if (scservo_map.count(joint)) {
            //    bool has_changed = !current_joint_values_.count(joint) || 
            //                      std::abs(current_joint_values_[joint] - value) > 0.01;
            //    if (has_changed) {
            //        RCLCPP_INFO(this->get_logger(), "SCServo Joint '%s': %.3f -> %.3f (ID %d)", 
            //                   joint.c_str(), original_value, value, scservo_map[joint]);
            //    }
            //}
            current_joint_values_[joint] = value;
        }
    }
    
    void timer_callback() {
        if (current_joint_values_.empty()) return;
        
        // Vor dem Loop: GroupSyncWrite-Parameter leeren
        dxl_groupSyncWrite_->clearParam();

        // SCServo Updates sammeln
        std::vector<u8> scservo_ids;
        std::vector<s16> scservo_positions;
        std::vector<u16> scservo_speeds;
        std::vector<u8> scservo_accs;
        
        int scservo_changes = 0;
        
        for (const auto& [joint, value] : current_joint_values_) {
            if (scservo_map.count(joint)) {
                // SCServo (debuggen)
                int servo_id = scservo_map[joint];
                int new_pos = rad_to_scservo(value);
                int last_pos = last_sent_positions_[joint];
                
                //if (new_pos != last_pos) {
                //    RCLCPP_INFO(this->get_logger(), "SCServo %s (ID %d): %d->%d", 
                //               joint.c_str(), servo_id, last_pos, new_pos);
                //    scservo_changes++;
                //}
                
                scservo_ids.push_back(servo_id);
                scservo_positions.push_back(new_pos);
                scservo_speeds.push_back(2400);
                scservo_accs.push_back(200);
                last_sent_positions_[joint] = new_pos;
            } else if (dynamixel_map.count(joint)) {
                int servo_id = dynamixel_map[joint];
                int new_pos = rad_to_dynamixel(value);

                // Logging hinzufügen
                //RCLCPP_INFO(this->get_logger(), "TIMER: Dynamixel %s (ID %d): %d", joint.c_str(), servo_id, new_pos);

                uint8_t param_goal_position[4];
                param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(new_pos));
                param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(new_pos));
                param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(new_pos));
                param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(new_pos));

                dxl_groupSyncWrite_->addParam(servo_id, param_goal_position);
                last_sent_positions_[joint] = new_pos;
            }
        }
        
        // Dynamixel SyncWrite senden
        if (!dynamixel_map.empty()) {
            int comm_result = dxl_groupSyncWrite_->txPacket();
            //RCLCPP_INFO(this->get_logger(), "Dynamixel SyncWrite gesendet.");
            if (comm_result != COMM_SUCCESS) {
                RCLCPP_ERROR(this->get_logger(), "Dynamixel SyncWrite fehlgeschlagen: %d", comm_result);
            }
        }
        
        // SCServo Batch senden
        if (!scservo_ids.empty()) {
            sm_st.SyncWritePosEx(scservo_ids.data(), scservo_ids.size(), 
                                scservo_positions.data(), scservo_speeds.data(), scservo_accs.data());
            //if (scservo_changes > 0) {
            //    RCLCPP_INFO(this->get_logger(), "Sent %d SCServo changes", scservo_changes);
            //}
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ServoMapperNode>();
    // PortHandler global setzen
    global_dxl_portHandler = node->getPortHandler(); // Du brauchst eine Getter-Funktion!
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}