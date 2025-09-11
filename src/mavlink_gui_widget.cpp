#include "mavlink_gui_tester/mavlink_gui_widget.hpp"
#include <QtWidgets/QApplication>
#include <QtWidgets/QMessageBox>
#include <QtCore/QDebug>
#include <chrono>
#include <cstring>
#include <algorithm>
#include <sstream>
#include <iomanip>

namespace mavlink_gui_tester {

MAVLinkGUIWidget::MAVLinkGUIWidget(std::shared_ptr<rclcpp::Node> node, QWidget *parent)
    : QMainWindow(parent),
      node_(node),
      serial_fd_(-1),
      running_(false),
      serial_port_("/dev/ttyUSB0"),
      baudrate_(115200),
      system_id_(255),
      component_id_(1),
      target_system_id_(1),
      target_component_id_(1),
      message_count_(0) {
    
    setupUI();
    
    // Initialize timer for updating UI
    update_timer_ = new QTimer(this);
    connect(update_timer_, &QTimer::timeout, this, &MAVLinkGUIWidget::updateReceivedMessages);
    connect(update_timer_, &QTimer::timeout, this, &MAVLinkGUIWidget::updateConnectionStatus);
    update_timer_->start(100); // Update every 100ms
    
    // Get parameters from ROS node if available
    if (node_) {
        try {
            serial_port_ = node_->get_parameter("serial_port").as_string();
            baudrate_ = node_->get_parameter("baudrate").as_int();
            system_id_ = node_->get_parameter("system_id").as_int();
            component_id_ = node_->get_parameter("component_id").as_int();
            target_system_id_ = node_->get_parameter("target_system_id").as_int();
            target_component_id_ = node_->get_parameter("target_component_id").as_int();
            
            // Update UI with parameters
            serial_port_edit_->setText(QString::fromStdString(serial_port_));
            system_id_spin_->setValue(system_id_);
            component_id_spin_->setValue(component_id_);
            target_system_id_spin_->setValue(target_system_id_);
            target_component_id_spin_->setValue(target_component_id_);
            
            // Set baudrate in combo box
            int index = baudrate_combo_->findText(QString::number(baudrate_));
            if (index != -1) {
                baudrate_combo_->setCurrentIndex(index);
            }
        } catch (const std::exception& e) {
            RCLCPP_WARN(node_->get_logger(), "Could not get parameters: %s", e.what());
        }
    }
    
    setWindowTitle("MAVLink GUI Tester");
    resize(800, 600);
}

MAVLinkGUIWidget::~MAVLinkGUIWidget() {
    running_ = false;
    if (rx_thread_.joinable()) {
        rx_thread_.join();
    }
    closeSerialPort();
}

void MAVLinkGUIWidget::setupUI() {
    // Create central widget and main layout
    QWidget* central_widget = new QWidget(this);
    setCentralWidget(central_widget);
    
    QVBoxLayout* main_layout = new QVBoxLayout(central_widget);
    
    // Create main tabs
    main_tabs_ = new QTabWidget(this);
    main_layout->addWidget(main_tabs_);
    
    // Setup different tabs
    setupConnectionTab();
    setupQuickCommandsTab();
    setupManualControlTab();
    setupCommandLongTab();
    setupRCChannelsTab();
    setupServoOutputTab();
    setupAttitudeTab();
    setupCustomHexTab();
    
    // Setup received messages area
    setupReceivedMessagesArea();
    main_layout->addWidget(received_messages_text_, 1);
    
    // Setup status bar
    status_bar_ = statusBar();
    status_label_ = new QLabel("Disconnected");
    message_count_label_ = new QLabel("Messages: 0");
    status_bar_->addWidget(status_label_);
    status_bar_->addPermanentWidget(message_count_label_);
}

void MAVLinkGUIWidget::setupConnectionTab() {
    connection_tab_ = new QWidget();
    main_tabs_->addTab(connection_tab_, "Connection");
    
    QVBoxLayout* layout = new QVBoxLayout(connection_tab_);
    
    // Connection settings group
    QGroupBox* settings_group = new QGroupBox("Serial Port Settings");
    QGridLayout* settings_layout = new QGridLayout(settings_group);
    
    // Serial port
    settings_layout->addWidget(new QLabel("Serial Port:"), 0, 0);
    serial_port_edit_ = new QLineEdit("/dev/ttyUSB0");
    settings_layout->addWidget(serial_port_edit_, 0, 1);
    
    // Baudrate
    settings_layout->addWidget(new QLabel("Baudrate:"), 1, 0);
    baudrate_combo_ = new QComboBox();
    baudrate_combo_->addItems({"9600", "19200", "38400", "57600", "115200", "230400", "460800", "921600"});
    baudrate_combo_->setCurrentText("115200");
    settings_layout->addWidget(baudrate_combo_, 1, 1);
    
    layout->addWidget(settings_group);
    
    // MAVLink settings group
    QGroupBox* mavlink_group = new QGroupBox("MAVLink Settings");
    QGridLayout* mavlink_layout = new QGridLayout(mavlink_group);
    
    // System ID
    mavlink_layout->addWidget(new QLabel("System ID:"), 0, 0);
    system_id_spin_ = new QSpinBox();
    system_id_spin_->setRange(1, 255);
    system_id_spin_->setValue(255);
    mavlink_layout->addWidget(system_id_spin_, 0, 1);
    
    // Component ID
    mavlink_layout->addWidget(new QLabel("Component ID:"), 1, 0);
    component_id_spin_ = new QSpinBox();
    component_id_spin_->setRange(1, 255);
    component_id_spin_->setValue(1);
    mavlink_layout->addWidget(component_id_spin_, 1, 1);
    
    // Target System ID
    mavlink_layout->addWidget(new QLabel("Target System ID:"), 0, 2);
    target_system_id_spin_ = new QSpinBox();
    target_system_id_spin_->setRange(1, 255);
    target_system_id_spin_->setValue(1);
    mavlink_layout->addWidget(target_system_id_spin_, 0, 3);
    
    // Target Component ID
    mavlink_layout->addWidget(new QLabel("Target Component ID:"), 1, 2);
    target_component_id_spin_ = new QSpinBox();
    target_component_id_spin_->setRange(1, 255);
    target_component_id_spin_->setValue(1);
    mavlink_layout->addWidget(target_component_id_spin_, 1, 3);
    
    layout->addWidget(mavlink_group);
    
    // Connection buttons
    QHBoxLayout* button_layout = new QHBoxLayout();
    connect_btn_ = new QPushButton("Connect");
    disconnect_btn_ = new QPushButton("Disconnect");
    disconnect_btn_->setEnabled(false);
    
    connect(connect_btn_, &QPushButton::clicked, this, &MAVLinkGUIWidget::onConnectSerial);
    connect(disconnect_btn_, &QPushButton::clicked, this, &MAVLinkGUIWidget::onDisconnectSerial);
    
    button_layout->addWidget(connect_btn_);
    button_layout->addWidget(disconnect_btn_);
    button_layout->addStretch();
    
    layout->addLayout(button_layout);
    
    // Connection status
    QGroupBox* status_group = new QGroupBox("Status");
    QVBoxLayout* status_layout = new QVBoxLayout(status_group);
    
    connection_status_label_ = new QLabel("Disconnected");
    connection_indicator_ = new QProgressBar();
    connection_indicator_->setRange(0, 1);
    connection_indicator_->setValue(0);
    
    status_layout->addWidget(connection_status_label_);
    status_layout->addWidget(connection_indicator_);
    
    layout->addWidget(status_group);
    layout->addStretch();
}

void MAVLinkGUIWidget::setupQuickCommandsTab() {
    quick_commands_tab_ = new QWidget();
    main_tabs_->addTab(quick_commands_tab_, "Quick Commands");
    
    QVBoxLayout* layout = new QVBoxLayout(quick_commands_tab_);
    
    QGroupBox* commands_group = new QGroupBox("Quick Commands");
    QGridLayout* commands_layout = new QGridLayout(commands_group);
    
    heartbeat_btn_ = new QPushButton("Send Heartbeat");
    heartbeat_btn_->setMinimumHeight(40);
    connect(heartbeat_btn_, &QPushButton::clicked, this, &MAVLinkGUIWidget::onSendHeartbeat);
    commands_layout->addWidget(heartbeat_btn_, 0, 0);
    
    clear_messages_btn_ = new QPushButton("Clear Messages");
    clear_messages_btn_->setMinimumHeight(40);
    connect(clear_messages_btn_, &QPushButton::clicked, this, &MAVLinkGUIWidget::onClearMessages);
    commands_layout->addWidget(clear_messages_btn_, 0, 1);
    
    layout->addWidget(commands_group);
    layout->addStretch();
}

void MAVLinkGUIWidget::setupManualControlTab() {
    manual_control_tab_ = new QWidget();
    main_tabs_->addTab(manual_control_tab_, "Manual Control");
    
    QVBoxLayout* layout = new QVBoxLayout(manual_control_tab_);
    
    QGroupBox* control_group = new QGroupBox("Manual Control Values");
    QGridLayout* control_layout = new QGridLayout(control_group);
    
    // X axis
    control_layout->addWidget(new QLabel("X (-1000 to 1000):"), 0, 0);
    manual_x_spin_ = new QSpinBox();
    manual_x_spin_->setRange(-1000, 1000);
    manual_x_spin_->setValue(0);
    control_layout->addWidget(manual_x_spin_, 0, 1);
    
    // Y axis
    control_layout->addWidget(new QLabel("Y (-1000 to 1000):"), 1, 0);
    manual_y_spin_ = new QSpinBox();
    manual_y_spin_->setRange(-1000, 1000);
    manual_y_spin_->setValue(0);
    control_layout->addWidget(manual_y_spin_, 1, 1);
    
    // Z axis
    control_layout->addWidget(new QLabel("Z (-1000 to 1000):"), 2, 0);
    manual_z_spin_ = new QSpinBox();
    manual_z_spin_->setRange(-1000, 1000);
    manual_z_spin_->setValue(0);
    control_layout->addWidget(manual_z_spin_, 2, 1);
    
    // R axis
    control_layout->addWidget(new QLabel("R (-1000 to 1000):"), 3, 0);
    manual_r_spin_ = new QSpinBox();
    manual_r_spin_->setRange(-1000, 1000);
    manual_r_spin_->setValue(0);
    control_layout->addWidget(manual_r_spin_, 3, 1);
    
    // Buttons
    control_layout->addWidget(new QLabel("Buttons (0-65535):"), 4, 0);
    manual_buttons_spin_ = new QSpinBox();
    manual_buttons_spin_->setRange(0, 65535);
    manual_buttons_spin_->setValue(0);
    control_layout->addWidget(manual_buttons_spin_, 4, 1);
    
    layout->addWidget(control_group);
    
    send_manual_control_btn_ = new QPushButton("Send Manual Control");
    send_manual_control_btn_->setMinimumHeight(40);
    connect(send_manual_control_btn_, &QPushButton::clicked, this, &MAVLinkGUIWidget::onSendManualControl);
    layout->addWidget(send_manual_control_btn_);
    
    layout->addStretch();
}

void MAVLinkGUIWidget::setupCommandLongTab() {
    command_long_tab_ = new QWidget();
    main_tabs_->addTab(command_long_tab_, "Command Long");
    
    QVBoxLayout* layout = new QVBoxLayout(command_long_tab_);
    
    QGroupBox* command_group = new QGroupBox("Command Long Parameters");
    QGridLayout* command_layout = new QGridLayout(command_group);
    
    // Command ID
    command_layout->addWidget(new QLabel("Command ID:"), 0, 0);
    command_id_spin_ = new QSpinBox();
    command_id_spin_->setRange(0, 65535);
    command_id_spin_->setValue(0);
    command_layout->addWidget(command_id_spin_, 0, 1);
    
    // Parameter 1
    command_layout->addWidget(new QLabel("Param1:"), 1, 0);
    param1_spin_ = new QDoubleSpinBox();
    param1_spin_->setRange(-999999.0, 999999.0);
    param1_spin_->setValue(0.0);
    param1_spin_->setDecimals(3);
    command_layout->addWidget(param1_spin_, 1, 1);
    
    // Parameter 2
    command_layout->addWidget(new QLabel("Param2:"), 2, 0);
    param2_spin_ = new QDoubleSpinBox();
    param2_spin_->setRange(-999999.0, 999999.0);
    param2_spin_->setValue(0.0);
    param2_spin_->setDecimals(3);
    command_layout->addWidget(param2_spin_, 2, 1);
    
    // Parameter 3
    command_layout->addWidget(new QLabel("Param3:"), 3, 0);
    param3_spin_ = new QDoubleSpinBox();
    param3_spin_->setRange(-999999.0, 999999.0);
    param3_spin_->setValue(0.0);
    param3_spin_->setDecimals(3);
    command_layout->addWidget(param3_spin_, 3, 1);
    
    // Parameter 4
    command_layout->addWidget(new QLabel("Param4:"), 4, 0);
    param4_spin_ = new QDoubleSpinBox();
    param4_spin_->setRange(-999999.0, 999999.0);
    param4_spin_->setValue(0.0);
    param4_spin_->setDecimals(3);
    command_layout->addWidget(param4_spin_, 4, 1);
    
    // Parameter 5
    command_layout->addWidget(new QLabel("Param5:"), 5, 0);
    param5_spin_ = new QDoubleSpinBox();
    param5_spin_->setRange(-999999.0, 999999.0);
    param5_spin_->setValue(0.0);
    param5_spin_->setDecimals(3);
    command_layout->addWidget(param5_spin_, 5, 1);
    
    // Parameter 6
    command_layout->addWidget(new QLabel("Param6:"), 6, 0);
    param6_spin_ = new QDoubleSpinBox();
    param6_spin_->setRange(-999999.0, 999999.0);
    param6_spin_->setValue(0.0);
    param6_spin_->setDecimals(3);
    command_layout->addWidget(param6_spin_, 6, 1);
    
    // Parameter 7
    command_layout->addWidget(new QLabel("Param7:"), 7, 0);
    param7_spin_ = new QDoubleSpinBox();
    param7_spin_->setRange(-999999.0, 999999.0);
    param7_spin_->setValue(0.0);
    param7_spin_->setDecimals(3);
    command_layout->addWidget(param7_spin_, 7, 1);
    
    layout->addWidget(command_group);
    
    send_command_long_btn_ = new QPushButton("Send Command Long");
    send_command_long_btn_->setMinimumHeight(40);
    connect(send_command_long_btn_, &QPushButton::clicked, this, &MAVLinkGUIWidget::onSendCommandLong);
    layout->addWidget(send_command_long_btn_);
    
    layout->addStretch();
}

void MAVLinkGUIWidget::setupRCChannelsTab() {
    rc_channels_tab_ = new QWidget();
    main_tabs_->addTab(rc_channels_tab_, "RC Channels");
    
    QVBoxLayout* layout = new QVBoxLayout(rc_channels_tab_);
    
    QGroupBox* rc_group = new QGroupBox("RC Channel Override (0-65535)");
    QGridLayout* rc_layout = new QGridLayout(rc_group);
    
    rc_channel_spins_.resize(8);
    for (int i = 0; i < 8; ++i) {
        rc_layout->addWidget(new QLabel(QString("Channel %1:").arg(i + 1)), i / 2, (i % 2) * 2);
        rc_channel_spins_[i] = new QSpinBox();
        rc_channel_spins_[i]->setRange(0, 65535);
        rc_channel_spins_[i]->setValue(1500); // Neutral position
        rc_layout->addWidget(rc_channel_spins_[i], i / 2, (i % 2) * 2 + 1);
    }
    
    layout->addWidget(rc_group);
    
    send_rc_channels_btn_ = new QPushButton("Send RC Channels Override");
    send_rc_channels_btn_->setMinimumHeight(40);
    connect(send_rc_channels_btn_, &QPushButton::clicked, this, &MAVLinkGUIWidget::onSendRCChannels);
    layout->addWidget(send_rc_channels_btn_);
    
    layout->addStretch();
}

void MAVLinkGUIWidget::setupServoOutputTab() {
    servo_output_tab_ = new QWidget();
    main_tabs_->addTab(servo_output_tab_, "Servo Output");
    
    QVBoxLayout* layout = new QVBoxLayout(servo_output_tab_);
    
    QGroupBox* servo_group = new QGroupBox("Servo Output Raw (0-65535)");
    QGridLayout* servo_layout = new QGridLayout(servo_group);
    
    servo_output_spins_.resize(8);
    for (int i = 0; i < 8; ++i) {
        servo_layout->addWidget(new QLabel(QString("Servo %1:").arg(i + 1)), i / 2, (i % 2) * 2);
        servo_output_spins_[i] = new QSpinBox();
        servo_output_spins_[i]->setRange(0, 65535);
        servo_output_spins_[i]->setValue(1500); // Neutral position
        servo_layout->addWidget(servo_output_spins_[i], i / 2, (i % 2) * 2 + 1);
    }
    
    layout->addWidget(servo_group);
    
    send_servo_output_btn_ = new QPushButton("Send Servo Output Raw");
    send_servo_output_btn_->setMinimumHeight(40);
    connect(send_servo_output_btn_, &QPushButton::clicked, this, &MAVLinkGUIWidget::onSendServoOutput);
    layout->addWidget(send_servo_output_btn_);
    
    layout->addStretch();
}

void MAVLinkGUIWidget::setupAttitudeTab() {
    attitude_tab_ = new QWidget();
    main_tabs_->addTab(attitude_tab_, "Attitude");
    
    QVBoxLayout* layout = new QVBoxLayout(attitude_tab_);
    
    QGroupBox* attitude_group = new QGroupBox("Attitude Values");
    QGridLayout* attitude_layout = new QGridLayout(attitude_group);
    
    // Roll
    attitude_layout->addWidget(new QLabel("Roll (rad):"), 0, 0);
    roll_spin_ = new QDoubleSpinBox();
    roll_spin_->setRange(-3.14159, 3.14159);
    roll_spin_->setValue(0.0);
    roll_spin_->setDecimals(3);
    roll_spin_->setSingleStep(0.1);
    attitude_layout->addWidget(roll_spin_, 0, 1);
    
    // Pitch
    attitude_layout->addWidget(new QLabel("Pitch (rad):"), 1, 0);
    pitch_spin_ = new QDoubleSpinBox();
    pitch_spin_->setRange(-3.14159, 3.14159);
    pitch_spin_->setValue(0.0);
    pitch_spin_->setDecimals(3);
    pitch_spin_->setSingleStep(0.1);
    attitude_layout->addWidget(pitch_spin_, 1, 1);
    
    // Yaw
    attitude_layout->addWidget(new QLabel("Yaw (rad):"), 2, 0);
    yaw_spin_ = new QDoubleSpinBox();
    yaw_spin_->setRange(-3.14159, 3.14159);
    yaw_spin_->setValue(0.0);
    yaw_spin_->setDecimals(3);
    yaw_spin_->setSingleStep(0.1);
    attitude_layout->addWidget(yaw_spin_, 2, 1);
    
    // Roll speed
    attitude_layout->addWidget(new QLabel("Roll Speed (rad/s):"), 0, 2);
    rollspeed_spin_ = new QDoubleSpinBox();
    rollspeed_spin_->setRange(-10.0, 10.0);
    rollspeed_spin_->setValue(0.0);
    rollspeed_spin_->setDecimals(3);
    rollspeed_spin_->setSingleStep(0.1);
    attitude_layout->addWidget(rollspeed_spin_, 0, 3);
    
    // Pitch speed
    attitude_layout->addWidget(new QLabel("Pitch Speed (rad/s):"), 1, 2);
    pitchspeed_spin_ = new QDoubleSpinBox();
    pitchspeed_spin_->setRange(-10.0, 10.0);
    pitchspeed_spin_->setValue(0.0);
    pitchspeed_spin_->setDecimals(3);
    pitchspeed_spin_->setSingleStep(0.1);
    attitude_layout->addWidget(pitchspeed_spin_, 1, 3);
    
    // Yaw speed
    attitude_layout->addWidget(new QLabel("Yaw Speed (rad/s):"), 2, 2);
    yawspeed_spin_ = new QDoubleSpinBox();
    yawspeed_spin_->setRange(-10.0, 10.0);
    yawspeed_spin_->setValue(0.0);
    yawspeed_spin_->setDecimals(3);
    yawspeed_spin_->setSingleStep(0.1);
    attitude_layout->addWidget(yawspeed_spin_, 2, 3);
    
    layout->addWidget(attitude_group);
    
    send_attitude_btn_ = new QPushButton("Send Attitude");
    send_attitude_btn_->setMinimumHeight(40);
    connect(send_attitude_btn_, &QPushButton::clicked, this, &MAVLinkGUIWidget::onSendAttitude);
    layout->addWidget(send_attitude_btn_);
    
    layout->addStretch();
}

void MAVLinkGUIWidget::setupCustomHexTab() {
    custom_hex_tab_ = new QWidget();
    main_tabs_->addTab(custom_hex_tab_, "Custom Hex");
    
    QVBoxLayout* layout = new QVBoxLayout(custom_hex_tab_);
    
    QGroupBox* hex_group = new QGroupBox("Raw Hex Data");
    QVBoxLayout* hex_layout = new QVBoxLayout(hex_group);
    
    QLabel* instruction_label = new QLabel("Enter raw hex data (e.g., 'FE 09 00 FF 01 00 00...'):");
    hex_layout->addWidget(instruction_label);
    
    hex_input_edit_ = new QTextEdit();
    hex_input_edit_->setMaximumHeight(100);
    hex_input_edit_->setPlaceholderText("FE 09 00 FF 01 00 00 00 00 00 00 00 03 93 90");
    hex_layout->addWidget(hex_input_edit_);
    
    layout->addWidget(hex_group);
    
    send_hex_btn_ = new QPushButton("Send Raw Hex Data");
    send_hex_btn_->setMinimumHeight(40);
    connect(send_hex_btn_, &QPushButton::clicked, this, &MAVLinkGUIWidget::onSendCustomHex);
    layout->addWidget(send_hex_btn_);
    
    layout->addStretch();
}

void MAVLinkGUIWidget::setupReceivedMessagesArea() {
    received_messages_text_ = new QTextEdit();
    received_messages_text_->setReadOnly(true);
    received_messages_text_->setMaximumHeight(200);
    received_messages_text_->setFont(QFont("Courier", 10));
    received_messages_text_->setPlaceholderText("Received messages will appear here...");
}

void MAVLinkGUIWidget::onConnectSerial() {
    // Update configuration from UI
    serial_port_ = serial_port_edit_->text().toStdString();
    baudrate_ = baudrate_combo_->currentText().toInt();
    system_id_ = system_id_spin_->value();
    component_id_ = component_id_spin_->value();
    target_system_id_ = target_system_id_spin_->value();
    target_component_id_ = target_component_id_spin_->value();
    
    if (openSerialPort()) {
        connect_btn_->setEnabled(false);
        disconnect_btn_->setEnabled(true);
        
        // Start RX thread
        running_ = true;
        rx_thread_ = std::thread(&MAVLinkGUIWidget::rxThread, this);
        
        QMessageBox::information(this, "Success", "Connected to " + QString::fromStdString(serial_port_));
    } else {
        QMessageBox::critical(this, "Error", "Failed to connect to " + QString::fromStdString(serial_port_));
    }
}

void MAVLinkGUIWidget::onDisconnectSerial() {
    running_ = false;
    if (rx_thread_.joinable()) {
        rx_thread_.join();
    }
    closeSerialPort();
    
    connect_btn_->setEnabled(true);
    disconnect_btn_->setEnabled(false);
    
    QMessageBox::information(this, "Disconnected", "Serial port disconnected");
}

void MAVLinkGUIWidget::onSendHeartbeat() {
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

void MAVLinkGUIWidget::onSendManualControl() {
    std::vector<uint8_t> payload(11);
    
    int16_t x = manual_x_spin_->value();
    int16_t y = manual_y_spin_->value();
    int16_t z = manual_z_spin_->value();
    int16_t r = manual_r_spin_->value();
    uint16_t buttons = manual_buttons_spin_->value();
    
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

void MAVLinkGUIWidget::onSendCommandLong() {
    std::vector<uint8_t> payload(33);
    
    uint16_t command = command_id_spin_->value();
    float param1 = param1_spin_->value();
    float param2 = param2_spin_->value();
    float param3 = param3_spin_->value();
    float param4 = param4_spin_->value();
    float param5 = param5_spin_->value();
    float param6 = param6_spin_->value();
    float param7 = param7_spin_->value();
    
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

void MAVLinkGUIWidget::onSendRCChannels() {
    std::vector<uint8_t> payload(22);
    
    // Pack RC channels as little-endian
    for (int i = 0; i < 8; ++i) {
        uint16_t value = rc_channel_spins_[i]->value();
        payload[i * 2] = value & 0xFF;
        payload[i * 2 + 1] = (value >> 8) & 0xFF;
    }
    
    // Add remaining channels (set to 0)
    for (int i = 16; i < 22; ++i) {
        payload[i] = 0;
    }
    
    payload[20] = target_system_id_;
    payload[21] = target_component_id_;
    
    sendMAVLinkMessage(70, payload);
}

void MAVLinkGUIWidget::onSendServoOutput() {
    std::vector<uint8_t> payload(21);
    
    // Timestamp (4 bytes) - set to 0
    payload[0] = payload[1] = payload[2] = payload[3] = 0;
    
    // Pack servo outputs as little-endian
    for (int i = 0; i < 8; ++i) {
        uint16_t value = servo_output_spins_[i]->value();
        payload[4 + i * 2] = value & 0xFF;
        payload[4 + i * 2 + 1] = (value >> 8) & 0xFF;
    }
    
    payload[20] = 0; // port (servo output port)
    
    sendMAVLinkMessage(36, payload);
}

void MAVLinkGUIWidget::onSendAttitude() {
    std::vector<uint8_t> payload(28);
    
    // Timestamp (4 bytes) - set to 0
    payload[0] = payload[1] = payload[2] = payload[3] = 0;
    
    float roll = roll_spin_->value();
    float pitch = pitch_spin_->value();
    float yaw = yaw_spin_->value();
    float rollspeed = rollspeed_spin_->value();
    float pitchspeed = pitchspeed_spin_->value();
    float yawspeed = yawspeed_spin_->value();
    
    // Pack floats as little-endian
    auto pack_float = [&](float value, size_t offset) {
        union { float f; uint32_t i; } converter;
        converter.f = value;
        payload[offset] = converter.i & 0xFF;
        payload[offset + 1] = (converter.i >> 8) & 0xFF;
        payload[offset + 2] = (converter.i >> 16) & 0xFF;
        payload[offset + 3] = (converter.i >> 24) & 0xFF;
    };
    
    pack_float(roll, 4);
    pack_float(pitch, 8);
    pack_float(yaw, 12);
    pack_float(rollspeed, 16);
    pack_float(pitchspeed, 20);
    pack_float(yawspeed, 24);
    
    sendMAVLinkMessage(30, payload);
}

void MAVLinkGUIWidget::onSendCustomHex() {
    QString hex_text = hex_input_edit_->toPlainText().trimmed();
    if (hex_text.isEmpty()) {
        QMessageBox::warning(this, "Warning", "Please enter hex data to send");
        return;
    }
    
    try {
        auto bytes = hexToBytes(hex_text.toStdString());
        if (serial_fd_ >= 0 && !bytes.empty()) {
            write(serial_fd_, bytes.data(), bytes.size());
            if (node_) {
                RCLCPP_INFO(node_->get_logger(), "Sent raw hex: %s", bytesToHex(bytes).c_str());
            }
        } else {
            QMessageBox::warning(this, "Warning", "Serial port not connected or invalid hex data");
        }
    } catch (const std::exception& e) {
        QMessageBox::critical(this, "Error", QString("Invalid hex data: %1").arg(e.what()));
    }
}

void MAVLinkGUIWidget::onClearMessages() {
    received_messages_text_->clear();
    message_count_ = 0;
}

void MAVLinkGUIWidget::updateReceivedMessages() {
    std::lock_guard<std::mutex> lock(rx_queue_mutex_);
    
    while (!rx_messages_.empty()) {
        received_messages_text_->append(QString::fromStdString(rx_messages_.front()));
        rx_messages_.pop();
        message_count_++;
    }
    
    // Auto-scroll to bottom
    QTextCursor cursor = received_messages_text_->textCursor();
    cursor.movePosition(QTextCursor::End);
    received_messages_text_->setTextCursor(cursor);
}

void MAVLinkGUIWidget::updateConnectionStatus() {
    if (serial_fd_ >= 0) {
        status_label_->setText("Connected to " + QString::fromStdString(serial_port_));
        connection_status_label_->setText("Connected");
        connection_indicator_->setValue(1);
    } else {
        status_label_->setText("Disconnected");
        connection_status_label_->setText("Disconnected");
        connection_indicator_->setValue(0);
    }
    
    message_count_label_->setText(QString("Messages: %1").arg(message_count_));
}

bool MAVLinkGUIWidget::openSerialPort() {
    serial_fd_ = open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (serial_fd_ < 0) {
        return false;
    }
    
    return configureSerialPort();
}

void MAVLinkGUIWidget::closeSerialPort() {
    if (serial_fd_ >= 0) {
        ::close(serial_fd_);
        serial_fd_ = -1;
    }
}

bool MAVLinkGUIWidget::configureSerialPort() {
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
        case 230400: speed = B230400; break;
        case 460800: speed = B460800; break;
        case 921600: speed = B921600; break;
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

void MAVLinkGUIWidget::rxThread() {
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

void MAVLinkGUIWidget::sendMAVLinkMessage(uint8_t msg_id, const std::vector<uint8_t>& payload) {
    if (serial_fd_ < 0) {
        QMessageBox::warning(this, "Warning", "Serial port not connected");
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
        if (node_) {
            RCLCPP_INFO(node_->get_logger(), "Sent MAVLink message ID %d: %s", msg_id, bytesToHex(message).c_str());
        }
    }
}

void MAVLinkGUIWidget::parseReceivedData(const std::vector<uint8_t>& data) {
    std::lock_guard<std::mutex> lock(rx_queue_mutex_);
    
    std::string hex_data = bytesToHex(data);
    auto now = std::chrono::steady_clock::now();
    auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
    
    rx_messages_.push("[" + std::to_string(timestamp) + "] RX: " + hex_data);
    
    // Keep only last 100 messages
    while (rx_messages_.size() > 100) {
        rx_messages_.pop();
    }
}

std::string MAVLinkGUIWidget::bytesToHex(const std::vector<uint8_t>& bytes) {
    std::stringstream ss;
    ss << std::hex << std::uppercase;
    for (size_t i = 0; i < bytes.size(); ++i) {
        if (i > 0) ss << " ";
        ss << std::setw(2) << std::setfill('0') << static_cast<unsigned>(bytes[i]);
    }
    return ss.str();
}

std::vector<uint8_t> MAVLinkGUIWidget::hexToBytes(const std::string& hex) {
    std::vector<uint8_t> bytes;
    std::string clean_hex;
    
    // Remove spaces and convert to uppercase
    for (char c : hex) {
        if (c != ' ' && c != '\t' && c != '\n' && c != '\r') {
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

} // namespace mavlink_gui_tester