#ifndef MAVLINK_GUI_NODE_HPP
#define MAVLINK_GUI_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <thread>
#include <atomic>
#include <mutex>
#include <queue>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <iostream>
#include <iomanip>
#include <sstream>

namespace mavlink_gui_tester {

struct MAVLinkCommand {
    uint8_t msg_id;
    std::string name;
    std::vector<std::pair<std::string, std::string>> params;
    std::string description;
};

class MAVLinkGUINode : public rclcpp::Node {
public:
    MAVLinkGUINode();
    ~MAVLinkGUINode();

private:
    // Serial port
    int serial_fd_;
    std::string serial_port_;
    int baudrate_;
    uint8_t system_id_;
    uint8_t component_id_;
    uint8_t target_system_id_;
    uint8_t target_component_id_;

    // Threading
    std::thread gui_thread_;
    std::thread rx_thread_;
    std::atomic<bool> running_;
    std::mutex console_mutex_;
    
    // Message queue
    std::queue<std::string> rx_messages_;
    std::mutex rx_queue_mutex_;
    
    // Available commands
    std::vector<MAVLinkCommand> available_commands_;
    
    // Terminal handling
    struct termios original_termios_;
    bool terminal_configured_;
    
    // Methods
    bool openSerialPort();
    void closeSerialPort();
    bool configureSerialPort();
    void configureTerminal();
    void restoreTerminal();
    
    void guiThread();
    void rxThread();
    
    void initializeCommands();
    void displayMenu();
    void displayReceivedMessages();
    void handleUserInput();
    void sendMAVLinkMessage(uint8_t msg_id, const std::vector<uint8_t>& payload);
    void sendHeartbeat();
    void sendManualControl(int16_t x, int16_t y, int16_t z, int16_t r, uint16_t buttons);
    void sendRCChannelsOverride(const std::array<uint16_t, 18>& channels);
    void sendCommandLong(uint16_t command, float param1 = 0, float param2 = 0, 
                        float param3 = 0, float param4 = 0, float param5 = 0, 
                        float param6 = 0, float param7 = 0);
    void sendServoOutputRaw(const std::array<uint16_t, 16>& servo_raw);
    void sendAttitude(float roll, float pitch, float yaw, float rollspeed, 
                     float pitchspeed, float yawspeed);
    
    std::string bytesToHex(const std::vector<uint8_t>& bytes);
    std::vector<uint8_t> hexToBytes(const std::string& hex);
    void clearScreen();
    void moveCursor(int row, int col);
    int getch();
    bool kbhit();
    
    // Message parsing
    void parseReceivedData(const std::vector<uint8_t>& data);
};

} // namespace mavlink_gui_tester

#endif // MAVLINK_GUI_NODE_HPP