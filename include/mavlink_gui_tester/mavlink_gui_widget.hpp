#ifndef MAVLINK_GUI_WIDGET_HPP
#define MAVLINK_GUI_WIDGET_HPP

#include <QtWidgets/QMainWindow>
#include <QtWidgets/QWidget>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QTextEdit>
#include <QtWidgets/QLabel>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QDoubleSpinBox>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QTableWidget>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QScrollArea>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QProgressBar>
#include <QtCore/QTimer>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <memory>
#include <thread>
#include <atomic>
#include <mutex>
#include <queue>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>

namespace mavlink_gui_tester {

class MAVLinkGUIWidget : public QMainWindow {
    Q_OBJECT

public:
    explicit MAVLinkGUIWidget(std::shared_ptr<rclcpp::Node> node, QWidget *parent = nullptr);
    ~MAVLinkGUIWidget();

private slots:
    void onConnectSerial();
    void onDisconnectSerial();
    void onSendHeartbeat();
    void onSendManualControl();
    void onSendCommandLong();
    void onSendRCChannels();
    void onSendServoOutput();
    void onSendAttitude();
    void onSendCustomHex();
    void onClearMessages();
    void updateReceivedMessages();
    void updateConnectionStatus();

private:
    void setupUI();
    void setupConnectionTab();
    void setupQuickCommandsTab();
    void setupManualControlTab();
    void setupCommandLongTab();
    void setupRCChannelsTab();
    void setupServoOutputTab();
    void setupAttitudeTab();
    void setupCustomHexTab();
    void setupReceivedMessagesArea();
    
    bool openSerialPort();
    void closeSerialPort();
    bool configureSerialPort();
    void rxThread();
    void sendMAVLinkMessage(uint8_t msg_id, const std::vector<uint8_t>& payload);
    void parseReceivedData(const std::vector<uint8_t>& data);
    std::string bytesToHex(const std::vector<uint8_t>& bytes);
    std::vector<uint8_t> hexToBytes(const std::string& hex);
    
    // ROS2 node
    std::shared_ptr<rclcpp::Node> node_;
    
    // Serial port
    int serial_fd_;
    std::atomic<bool> running_;
    std::thread rx_thread_;
    std::mutex rx_queue_mutex_;
    std::queue<std::string> rx_messages_;
    
    // UI Components
    QTabWidget* main_tabs_;
    
    // Connection tab
    QWidget* connection_tab_;
    QLineEdit* serial_port_edit_;
    QComboBox* baudrate_combo_;
    QSpinBox* system_id_spin_;
    QSpinBox* component_id_spin_;
    QSpinBox* target_system_id_spin_;
    QSpinBox* target_component_id_spin_;
    QPushButton* connect_btn_;
    QPushButton* disconnect_btn_;
    QLabel* connection_status_label_;
    QProgressBar* connection_indicator_;
    
    // Quick commands tab
    QWidget* quick_commands_tab_;
    QPushButton* heartbeat_btn_;
    QPushButton* clear_messages_btn_;
    
    // Manual control tab
    QWidget* manual_control_tab_;
    QSpinBox* manual_x_spin_;
    QSpinBox* manual_y_spin_;
    QSpinBox* manual_z_spin_;
    QSpinBox* manual_r_spin_;
    QSpinBox* manual_buttons_spin_;
    QPushButton* send_manual_control_btn_;
    
    // Command Long tab
    QWidget* command_long_tab_;
    QSpinBox* command_id_spin_;
    QDoubleSpinBox* param1_spin_;
    QDoubleSpinBox* param2_spin_;
    QDoubleSpinBox* param3_spin_;
    QDoubleSpinBox* param4_spin_;
    QDoubleSpinBox* param5_spin_;
    QDoubleSpinBox* param6_spin_;
    QDoubleSpinBox* param7_spin_;
    QPushButton* send_command_long_btn_;
    
    // RC Channels tab
    QWidget* rc_channels_tab_;
    std::vector<QSpinBox*> rc_channel_spins_;
    QPushButton* send_rc_channels_btn_;
    
    // Servo Output tab
    QWidget* servo_output_tab_;
    std::vector<QSpinBox*> servo_output_spins_;
    QPushButton* send_servo_output_btn_;
    
    // Attitude tab
    QWidget* attitude_tab_;
    QDoubleSpinBox* roll_spin_;
    QDoubleSpinBox* pitch_spin_;
    QDoubleSpinBox* yaw_spin_;
    QDoubleSpinBox* rollspeed_spin_;
    QDoubleSpinBox* pitchspeed_spin_;
    QDoubleSpinBox* yawspeed_spin_;
    QPushButton* send_attitude_btn_;
    
    // Custom hex tab
    QWidget* custom_hex_tab_;
    QTextEdit* hex_input_edit_;
    QPushButton* send_hex_btn_;
    
    // Received messages area
    QTextEdit* received_messages_text_;
    QTimer* update_timer_;
    
    // Status bar
    QStatusBar* status_bar_;
    QLabel* status_label_;
    QLabel* message_count_label_;
    
    // Configuration
    std::string serial_port_;
    int baudrate_;
    uint8_t system_id_;
    uint8_t component_id_;
    uint8_t target_system_id_;
    uint8_t target_component_id_;
    
    int message_count_;
};

} // namespace mavlink_gui_tester

#endif // MAVLINK_GUI_WIDGET_HPP