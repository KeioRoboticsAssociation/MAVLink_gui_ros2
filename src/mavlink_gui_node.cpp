#include "mavlink_gui_tester/mavlink_gui_node.hpp"
#include <chrono>
#include <cstring>
#include <algorithm>
#include <sys/select.h>

namespace mavlink_gui_tester {

MAVLinkGUINode::MAVLinkGUINode() 
    : Node("mavlink_gui_node"),
      serial_fd_(-1),
      running_(false),
      terminal_configured_(false) {
    
    // Declare parameters
    this->declare_parameter("serial_port", "/dev/ttyUSB0");
    this->declare_parameter("baudrate", 115200);
    this->declare_parameter("system_id", 255);
    this->declare_parameter("component_id", 1);
    this->declare_parameter("target_system_id", 1);
    this->declare_parameter("target_component_id", 1);
    
    // Get parameters
    serial_port_ = this->get_parameter("serial_port").as_string();
    baudrate_ = this->get_parameter("baudrate").as_int();
    system_id_ = this->get_parameter("system_id").as_int();
    component_id_ = this->get_parameter("component_id").as_int();
    target_system_id_ = this->get_parameter("target_system_id").as_int();
    target_component_id_ = this->get_parameter("target_component_id").as_int();
    
    // Initialize available commands
    initializeCommands();
    
    // Configure terminal
    configureTerminal();
    
    // Open serial port
    if (!openSerialPort()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", serial_port_.c_str());
        RCLCPP_INFO(this->get_logger(), "GUI will run without serial connection");
    }
    
    // Start threads
    running_ = true;
    rx_thread_ = std::thread(&MAVLinkGUINode::rxThread, this);
    gui_thread_ = std::thread(&MAVLinkGUINode::guiThread, this);
    
    RCLCPP_INFO(this->get_logger(), "MAVLink GUI Tester started");
}

MAVLinkGUINode::~MAVLinkGUINode() {
    running_ = false;
    
    if (gui_thread_.joinable()) {
        gui_thread_.join();
    }
    if (rx_thread_.joinable()) {
        rx_thread_.join();
    }
    
    closeSerialPort();
    restoreTerminal();
}

bool MAVLinkGUINode::openSerialPort() {
    serial_fd_ = open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (serial_fd_ < 0) {
        return false;
    }
    
    return configureSerialPort();
}

void MAVLinkGUINode::closeSerialPort() {
    if (serial_fd_ >= 0) {
        close(serial_fd_);
        serial_fd_ = -1;
    }
}

bool MAVLinkGUINode::configureSerialPort() {
    struct termios tty;
    if (tcgetattr(serial_fd_, &tty) != 0) {
        return false;
    }
    
    // Set baud rate
    speed_t speed;
    switch (baudrate_) {
        case 9600: speed = B9600; break;
        case 19200: speed = B19200; break;
        case 38400: speed = B38400; break;
        case 57600: speed = B57600; break;
        case 115200: speed = B115200; break;
        default: speed = B115200; break;
    }
    
    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);
    
    // Configure 8N1
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;
    
    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO;
    tty.c_lflag &= ~ECHOE;
    tty.c_lflag &= ~ECHONL;
    tty.c_lflag &= ~ISIG;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);
    
    tty.c_oflag &= ~OPOST;
    tty.c_oflag &= ~ONLCR;
    
    tty.c_cc[VTIME] = 1;
    tty.c_cc[VMIN] = 0;
    
    if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
        return false;
    }
    
    return true;
}

void MAVLinkGUINode::configureTerminal() {
    tcgetattr(STDIN_FILENO, &original_termios_);
    
    struct termios raw = original_termios_;
    raw.c_lflag &= ~(ECHO | ICANON);
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 1;
    
    tcsetattr(STDIN_FILENO, TCSAFLUSH, &raw);
    terminal_configured_ = true;
}

void MAVLinkGUINode::restoreTerminal() {
    if (terminal_configured_) {
        tcsetattr(STDIN_FILENO, TCSAFLUSH, &original_termios_);
        terminal_configured_ = false;
    }
}

void MAVLinkGUINode::initializeCommands() {
    available_commands_ = {
        {0, "HEARTBEAT", {{"type", "0-255"}, {"autopilot", "0-255"}, {"base_mode", "0-255"}, {"custom_mode", "0-4294967295"}, {"system_status", "0-255"}, {"mavlink_version", "3"}}, "System heartbeat"},
        {11, "SET_MODE", {{"target_system", "1-255"}, {"base_mode", "0-255"}, {"custom_mode", "0-4294967295"}}, "Set system mode"},
        {69, "MANUAL_CONTROL", {{"x", "-1000 to 1000"}, {"y", "-1000 to 1000"}, {"z", "-1000 to 1000"}, {"r", "-1000 to 1000"}, {"buttons", "0-65535"}}, "Manual control input"},
        {70, "RC_CHANNELS_OVERRIDE", {{"chan1_raw", "0-65535"}, {"chan2_raw", "0-65535"}, {"chan3_raw", "0-65535"}, {"chan4_raw", "0-65535"}, {"chan5_raw", "0-65535"}, {"chan6_raw", "0-65535"}, {"chan7_raw", "0-65535"}, {"chan8_raw", "0-65535"}}, "RC channel override"},
        {76, "COMMAND_LONG", {{"command", "0-65535"}, {"param1", "float"}, {"param2", "float"}, {"param3", "float"}, {"param4", "float"}, {"param5", "float"}, {"param6", "float"}, {"param7", "float"}}, "Send command"},
        {36, "SERVO_OUTPUT_RAW", {{"servo1_raw", "0-65535"}, {"servo2_raw", "0-65535"}, {"servo3_raw", "0-65535"}, {"servo4_raw", "0-65535"}, {"servo5_raw", "0-65535"}, {"servo6_raw", "0-65535"}, {"servo7_raw", "0-65535"}, {"servo8_raw", "0-65535"}}, "Raw servo output"},
        {30, "ATTITUDE", {{"roll", "float"}, {"pitch", "float"}, {"yaw", "float"}, {"rollspeed", "float"}, {"pitchspeed", "float"}, {"yawspeed", "float"}}, "Vehicle attitude"}
    };
}

void MAVLinkGUINode::guiThread() {
    while (running_) {
        clearScreen();
        displayMenu();
        displayReceivedMessages();
        handleUserInput();
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

void MAVLinkGUINode::rxThread() {
    std::vector<uint8_t> buffer(1024);
    
    while (running_) {
        if (serial_fd_ >= 0) {
            ssize_t bytes_read = read(serial_fd_, buffer.data(), buffer.size());
            if (bytes_read > 0) {
                std::vector<uint8_t> data(buffer.begin(), buffer.begin() + bytes_read);
                parseReceivedData(data);
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void MAVLinkGUINode::displayMenu() {
    std::lock_guard<std::mutex> lock(console_mutex_);
    
    std::cout << "=== MAVLink GUI Tester ===" << std::endl;
    std::cout << "Serial Port: " << serial_port_ << " (" << (serial_fd_ >= 0 ? "Connected" : "Disconnected") << ")" << std::endl;
    std::cout << "System ID: " << (int)system_id_ << ", Component ID: " << (int)component_id_ << std::endl;
    std::cout << "Target System: " << (int)target_system_id_ << ", Target Component: " << (int)target_component_id_ << std::endl;
    std::cout << std::endl;
    
    std::cout << "Available Commands:" << std::endl;
    for (size_t i = 0; i < available_commands_.size(); ++i) {
        const auto& cmd = available_commands_[i];
        std::cout << i + 1 << ". " << cmd.name << " (ID: " << (int)cmd.msg_id << ") - " << cmd.description << std::endl;
    }
    
    std::cout << std::endl;
    std::cout << "Commands:" << std::endl;
    std::cout << "1-" << available_commands_.size() << ": Send MAVLink message" << std::endl;
    std::cout << "h: Send heartbeat" << std::endl;
    std::cout << "r: Send raw hex data" << std::endl;
    std::cout << "q: Quit" << std::endl;
    std::cout << std::endl;
    std::cout << "Press a key: ";
}

void MAVLinkGUINode::displayReceivedMessages() {
    std::lock_guard<std::mutex> rx_lock(rx_queue_mutex_);
    std::lock_guard<std::mutex> console_lock(console_mutex_);
    
    if (!rx_messages_.empty()) {
        std::cout << std::endl << "=== Received Messages ===" << std::endl;
        int count = 0;
        std::queue<std::string> temp_queue = rx_messages_;
        while (!temp_queue.empty() && count < 10) {
            std::cout << temp_queue.front() << std::endl;
            temp_queue.pop();
            count++;
        }
        std::cout << "=========================" << std::endl;
    }
}

void MAVLinkGUINode::handleUserInput() {
    if (kbhit()) {
        int ch = getch();
        
        if (ch == 'q' || ch == 'Q') {
            running_ = false;
            return;
        }
        
        if (ch == 'h' || ch == 'H') {
            sendHeartbeat();
            return;
        }
        
        if (ch == 'r' || ch == 'R') {
            std::lock_guard<std::mutex> lock(console_mutex_);
            std::cout << std::endl << "Enter raw hex data (e.g., 'FE 09 00 FF 01 00 00...'): ";
            std::string hex_input;
            std::getline(std::cin, hex_input);
            
            try {
                auto bytes = hexToBytes(hex_input);
                if (serial_fd_ >= 0 && !bytes.empty()) {
                    write(serial_fd_, bytes.data(), bytes.size());
                    std::cout << "Sent: " << bytesToHex(bytes) << std::endl;
                }
            } catch (const std::exception& e) {
                std::cout << "Error: " << e.what() << std::endl;
            }
            
            std::cout << "Press any key to continue...";
            getch();
            return;
        }
        
        // Handle numbered commands
        if (ch >= '1' && ch <= '9') {
            int cmd_idx = ch - '1';
            if (cmd_idx < static_cast<int>(available_commands_.size())) {
                const auto& cmd = available_commands_[cmd_idx];
                
                std::lock_guard<std::mutex> lock(console_mutex_);
                std::cout << std::endl << "=== " << cmd.name << " ===" << std::endl;
                std::cout << cmd.description << std::endl;
                
                std::vector<std::string> values;
                for (const auto& param : cmd.params) {
                    std::cout << param.first << " (" << param.second << "): ";
                    std::string value;
                    std::getline(std::cin, value);
                    values.push_back(value);
                }
                
                // Send the message based on command type
                if (cmd.msg_id == 0) { // HEARTBEAT
                    sendHeartbeat();
                } else if (cmd.msg_id == 69) { // MANUAL_CONTROL
                    if (values.size() >= 5) {
                        sendManualControl(
                            static_cast<int16_t>(std::stoi(values[0])),
                            static_cast<int16_t>(std::stoi(values[1])),
                            static_cast<int16_t>(std::stoi(values[2])),
                            static_cast<int16_t>(std::stoi(values[3])),
                            static_cast<uint16_t>(std::stoi(values[4]))
                        );
                    }
                } else if (cmd.msg_id == 76) { // COMMAND_LONG
                    if (values.size() >= 8) {
                        sendCommandLong(
                            static_cast<uint16_t>(std::stoi(values[0])),
                            std::stof(values[1]),
                            std::stof(values[2]),
                            std::stof(values[3]),
                            std::stof(values[4]),
                            std::stof(values[5]),
                            std::stof(values[6]),
                            std::stof(values[7])
                        );
                    }
                }
                
                std::cout << "Message sent! Press any key to continue...";
                getch();
            }
        }
    }
}

void MAVLinkGUINode::sendHeartbeat() {
    std::vector<uint8_t> payload(9, 0);
    payload[0] = 0;    // type: MAV_TYPE_GENERIC
    payload[1] = 0;    // autopilot: MAV_AUTOPILOT_GENERIC
    payload[2] = 0;    // base_mode
    payload[3] = 0;    // custom_mode (4 bytes)
    payload[4] = 0;
    payload[5] = 0;
    payload[6] = 0;
    payload[7] = 0;    // system_status: MAV_STATE_UNINIT
    payload[8] = 3;    // mavlink_version
    
    sendMAVLinkMessage(0, payload);
}

void MAVLinkGUINode::sendMAVLinkMessage(uint8_t msg_id, const std::vector<uint8_t>& payload) {
    if (serial_fd_ < 0) {
        RCLCPP_WARN(this->get_logger(), "Serial port not connected");
        return;
    }
    
    std::vector<uint8_t> message;
    message.push_back(0xFE);  // Start marker
    message.push_back(payload.size());  // Payload length
    message.push_back(0);     // Packet sequence
    message.push_back(system_id_);       // System ID
    message.push_back(component_id_);    // Component ID
    message.push_back(msg_id);           // Message ID
    
    // Add payload
    message.insert(message.end(), payload.begin(), payload.end());
    
    // Calculate checksum
    uint16_t checksum = 0xFFFF;
    for (size_t i = 1; i < message.size(); ++i) {
        checksum ^= message[i];
        for (int j = 0; j < 8; ++j) {
            if (checksum & 1) {
                checksum = (checksum >> 1) ^ 0xA001;
            } else {
                checksum >>= 1;
            }
        }
    }
    
    message.push_back(checksum & 0xFF);
    message.push_back((checksum >> 8) & 0xFF);
    
    ssize_t written = write(serial_fd_, message.data(), message.size());
    if (written > 0) {
        RCLCPP_INFO(this->get_logger(), "Sent MAVLink message ID %d: %s", msg_id, bytesToHex(message).c_str());
    }
}

void MAVLinkGUINode::sendManualControl(int16_t x, int16_t y, int16_t z, int16_t r, uint16_t buttons) {
    std::vector<uint8_t> payload(11);
    
    // Pack values as little-endian
    payload[0] = x & 0xFF;
    payload[1] = (x >> 8) & 0xFF;
    payload[2] = y & 0xFF;
    payload[3] = (y >> 8) & 0xFF;
    payload[4] = z & 0xFF;
    payload[5] = (z >> 8) & 0xFF;
    payload[6] = r & 0xFF;
    payload[7] = (r >> 8) & 0xFF;
    payload[8] = buttons & 0xFF;
    payload[9] = (buttons >> 8) & 0xFF;
    payload[10] = target_system_id_;
    
    sendMAVLinkMessage(69, payload);
}

void MAVLinkGUINode::sendCommandLong(uint16_t command, float param1, float param2, 
                                    float param3, float param4, float param5, 
                                    float param6, float param7) {
    std::vector<uint8_t> payload(33);
    
    // Pack floats and integers as little-endian
    auto pack_float = [&](float value, size_t offset) {
        union { float f; uint32_t i; } converter;
        converter.f = value;
        payload[offset] = converter.i & 0xFF;
        payload[offset + 1] = (converter.i >> 8) & 0xFF;
        payload[offset + 2] = (converter.i >> 16) & 0xFF;
        payload[offset + 3] = (converter.i >> 24) & 0xFF;
    };
    
    pack_float(param1, 0);
    pack_float(param2, 4);
    pack_float(param3, 8);
    pack_float(param4, 12);
    pack_float(param5, 16);
    pack_float(param6, 20);
    pack_float(param7, 24);
    
    payload[28] = command & 0xFF;
    payload[29] = (command >> 8) & 0xFF;
    payload[30] = target_system_id_;
    payload[31] = target_component_id_;
    payload[32] = 0; // confirmation
    
    sendMAVLinkMessage(76, payload);
}

std::string MAVLinkGUINode::bytesToHex(const std::vector<uint8_t>& bytes) {
    std::stringstream ss;
    ss << std::hex << std::uppercase;
    for (size_t i = 0; i < bytes.size(); ++i) {
        if (i > 0) ss << " ";
        ss << std::setw(2) << std::setfill('0') << static_cast<unsigned>(bytes[i]);
    }
    return ss.str();
}

std::vector<uint8_t> MAVLinkGUINode::hexToBytes(const std::string& hex) {
    std::vector<uint8_t> bytes;
    std::string clean_hex;
    
    // Remove spaces and convert to uppercase
    for (char c : hex) {
        if (c != ' ' && c != '\t' && c != '\n') {
            clean_hex += std::toupper(c);
        }
    }
    
    for (size_t i = 0; i < clean_hex.length(); i += 2) {
        if (i + 1 < clean_hex.length()) {
            std::string byte_str = clean_hex.substr(i, 2);
            uint8_t byte = static_cast<uint8_t>(std::stoul(byte_str, nullptr, 16));
            bytes.push_back(byte);
        }
    }
    
    return bytes;
}

void MAVLinkGUINode::parseReceivedData(const std::vector<uint8_t>& data) {
    std::lock_guard<std::mutex> lock(rx_queue_mutex_);
    
    std::string hex_data = bytesToHex(data);
    std::string timestamp = std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
    
    rx_messages_.push("[" + timestamp + "] RX: " + hex_data);
    
    // Keep only last 50 messages
    while (rx_messages_.size() > 50) {
        rx_messages_.pop();
    }
}

void MAVLinkGUINode::clearScreen() {
    std::cout << "\033[2J\033[H";
}

void MAVLinkGUINode::moveCursor(int row, int col) {
    std::cout << "\033[" << row << ";" << col << "H";
}

int MAVLinkGUINode::getch() {
    return getchar();
}

bool MAVLinkGUINode::kbhit() {
    struct timeval tv = {0, 0};
    fd_set readfds;
    
    FD_ZERO(&readfds);
    FD_SET(STDIN_FILENO, &readfds);
    
    return select(STDIN_FILENO + 1, &readfds, nullptr, nullptr, &tv) > 0;
}

} // namespace mavlink_gui_tester