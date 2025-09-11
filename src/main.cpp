#include <rclcpp/rclcpp.hpp>
#include <QtWidgets/QApplication>
#include "mavlink_gui_tester/mavlink_gui_widget.hpp"
#include <memory>
#include <thread>

int main(int argc, char ** argv) {
    // Initialize ROS2
    rclcpp::init(argc, argv);
    
    // Initialize Qt Application
    QApplication app(argc, argv);
    
    try {
        // Create ROS2 node
        auto node = std::make_shared<rclcpp::Node>("mavlink_gui_tester");
        
        // Declare parameters
        node->declare_parameter("serial_port", "/dev/ttyUSB0");
        node->declare_parameter("baudrate", 115200);
        node->declare_parameter("system_id", 255);
        node->declare_parameter("component_id", 1);
        node->declare_parameter("target_system_id", 1);
        node->declare_parameter("target_component_id", 1);
        
        // Create and show GUI
        mavlink_gui_tester::MAVLinkGUIWidget widget(node);
        widget.show();
        
        // Run ROS2 spinning in a separate thread
        std::thread ros_thread([&]() {
            rclcpp::spin(node);
        });
        
        // Run Qt event loop
        int result = app.exec();
        
        // Cleanup
        rclcpp::shutdown();
        if (ros_thread.joinable()) {
            ros_thread.join();
        }
        
        return result;
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        rclcpp::shutdown();
        return 1;
    }
}