# MAVLink GUI Tester

A ROS2 package that provides a graphical Qt-based GUI for testing MAVLink communication by sending arbitrary MAVLink messages manually.

## Features

- **Graphical Qt interface** with tabs and buttons
- Send predefined MAVLink messages (HEARTBEAT, MANUAL_CONTROL, COMMAND_LONG, etc.)
- **Connection tab** for configuring serial port settings
- **Quick commands** for common operations
- **Manual control** with input fields for X, Y, Z, R axes and buttons
- **Command Long** with 7 parameter inputs
- **RC Channels Override** for up to 8 channels  
- **Servo Output** for controlling servos
- **Attitude** control with roll, pitch, yaw inputs
- **Custom hex data** input for raw packet sending
- Real-time display of received messages
- Connection status indicator
- Message counter

## Prerequisites

- ROS2 (tested with Jazzy)
- Qt5 development libraries (`qtbase5-dev`, `libqt5widgets5`)
- Serial port access (typically `/dev/ttyUSB0` or `/dev/ttyACM0`)
- X11 display for GUI (or VNC/remote desktop)

## Building

```bash
cd ~/ros2_jazzy
colcon build --packages-select mavlink_gui_tester
source install/setup.bash
```

## Usage

### Basic usage with default parameters:
```bash
ros2 run mavlink_gui_tester mavlink_gui_tester_node
```

### Using launch file with custom parameters:
```bash
ros2 launch mavlink_gui_tester mavlink_gui_tester.launch.py serial_port:=/dev/ttyUSB0 baudrate:=115200
```

### Available launch parameters:
- `serial_port`: Serial port device (default: `/dev/ttyUSB0`)
- `baudrate`: Baud rate (default: `115200`)
- `system_id`: MAVLink system ID (default: `255`)
- `component_id`: MAVLink component ID (default: `1`)
- `target_system_id`: Target system ID (default: `1`)
- `target_component_id`: Target component ID (default: `1`)

## GUI Interface

The Qt GUI provides several tabs for different functionalities:

### 1. Connection Tab
- Configure serial port (default: `/dev/ttyUSB0`)
- Set baud rate (9600 to 921600)
- Configure MAVLink system/component IDs
- Connect/disconnect buttons
- Connection status indicator

### 2. Quick Commands Tab
- **Send Heartbeat**: Quick heartbeat message
- **Clear Messages**: Clear received message display

### 3. Manual Control Tab
- Input fields for X, Y, Z, R axes (-1000 to 1000)
- Button state input (0-65535)
- Send button to transmit manual control command

### 4. Command Long Tab
- Command ID input (0-65535)
- 7 parameter inputs (floating point)
- Send button for command transmission

### 5. RC Channels Tab
- 8 RC channel inputs (0-65535, default 1500)
- Send RC channel override command

### 6. Servo Output Tab
- 8 servo output inputs (0-65535, default 1500)
- Send servo output raw command

### 7. Attitude Tab
- Roll, pitch, yaw inputs (radians, -π to π)
- Roll, pitch, yaw speed inputs (rad/s)
- Send attitude message

### 8. Custom Hex Tab
- Raw hex data input field
- Send arbitrary hex packets directly

## Supported MAVLink Messages

The GUI supports sending the following MAVLink messages:

- **HEARTBEAT (ID: 0)**: System heartbeat
- **SET_MODE (ID: 11)**: Set system mode
- **MANUAL_CONTROL (ID: 69)**: Manual control input
- **RC_CHANNELS_OVERRIDE (ID: 70)**: RC channel override
- **COMMAND_LONG (ID: 76)**: Send command
- **SERVO_OUTPUT_RAW (ID: 36)**: Raw servo output
- **ATTITUDE (ID: 30)**: Vehicle attitude

## Example Usage

1. Connect your STM32 device to `/dev/ttyUSB0`
2. Launch the GUI: `ros2 run mavlink_gui_tester mavlink_gui_tester_node`
3. **Connection Tab**: Configure serial port settings and click "Connect"
4. **Quick Commands Tab**: Click "Send Heartbeat" to test basic communication
5. **Manual Control Tab**: Enter axis values (e.g., X=500, Y=200) and click "Send Manual Control"
6. **Custom Hex Tab**: Enter raw hex like "FE 09 00 FF 01 00..." and click "Send Raw Hex Data"
7. View received messages in the bottom text area
8. Monitor connection status in the status bar

## Troubleshooting

- **Serial port not found**: Check that your device is connected and the port exists (`ls /dev/tty*`)
- **Permission denied**: Add your user to the dialout group: `sudo usermod -a -G dialout $USER` (then logout/login)
- **GUI not displaying**: Ensure you have X11 display available (check `echo $DISPLAY`)
- **Qt errors**: Make sure Qt5 libraries are installed: `sudo apt install qtbase5-dev libqt5widgets5`
- **Build fails**: Ensure all dependencies are installed and colcon build is run from the workspace root

## Integration with STM32 MAVLink Interface

This GUI tester is designed to work alongside the `stm32_mavlink_interface` package. You can use it to:

- Test communication with STM32 devices
- Send commands to servo controllers
- Test encoder interfaces
- Debug RoboMaster controller communication
- Verify MAVLink message parsing and handling

## RoboMaster Controller Communication Examples

The GUI is particularly useful for debugging RoboMaster controller communication. Here are specific examples:

### 1. Testing RoboMaster Motor Control

**Scenario**: Debug individual motor control commands for RoboMaster M3508 motors

**Steps**:
1. **Connection Tab**: Connect to STM32 device (`/dev/ttyUSB0`, 115200 baud)
2. **Command Long Tab**: Send motor control commands
   - Command ID: `31010` (Custom RoboMaster motor command)
   - Param1: `1` (Motor ID 1-4)
   - Param2: `1500` (Speed in RPM, -3000 to 3000)
   - Param3: `0` (Torque limit, 0=default)
   - Click "Send Command Long"

**Expected Response**: Motor should rotate at specified RPM, received messages should show telemetry feedback

### 2. RoboMaster CAN Bus Diagnostics

**Scenario**: Verify CAN bus communication with RoboMaster ESCs

**Steps**:
1. **Custom Hex Tab**: Send raw CAN diagnostic packet
   ```
   FE 21 00 FF 01 4C 20 01 02 03 04 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 B7 3F
   ```
   - This sends a MAVLink message ID 76 (COMMAND_LONG) with RoboMaster CAN diagnostic command
2. **Monitor received messages** for CAN bus status responses

**Expected Response**: CAN device enumeration and status messages

### 3. RoboMaster Gimbal Control Testing

**Scenario**: Test 2-axis gimbal control (pitch/yaw)

**Steps**:
1. **Manual Control Tab**:
   - X: `0` (not used for gimbal)
   - Y: `500` (Pitch control, -1000 to 1000)
   - Z: `0` (not used for gimbal)  
   - R: `-300` (Yaw control, -1000 to 1000)
   - Buttons: `1` (Enable gimbal mode)
   - Click "Send Manual Control"

**Expected Response**: Gimbal should move to specified pitch/yaw positions

### 4. RoboMaster Chassis Movement

**Scenario**: Test omnidirectional chassis movement

**Steps**:
1. **RC Channels Tab**: Map movement to RC channels
   - Channel 1: `1700` (Forward/backward, 1000-2000, center=1500)
   - Channel 2: `1300` (Left/right strafe)
   - Channel 3: `1500` (Up/down - not used for chassis)
   - Channel 4: `1600` (Rotation)
   - Channels 5-8: `1500` (Neutral)
   - Click "Send RC Channels Override"

**Expected Response**: Chassis should move forward-right with slight rotation

### 5. RoboMaster Sensor Data Debugging

**Scenario**: Request and monitor sensor telemetry data

**Steps**:
1. **Command Long Tab**: Request sensor data stream
   - Command ID: `511` (MAV_CMD_SET_MESSAGE_INTERVAL)
   - Param1: `30` (ATTITUDE message ID)
   - Param2: `100000` (Interval in microseconds, 10Hz)
   - Click "Send Command Long"
2. **Monitor received messages** for attitude telemetry

**Expected Response**: Continuous stream of attitude data from IMU

### 6. RoboMaster Emergency Stop Testing

**Scenario**: Test emergency stop functionality

**Steps**:
1. **Quick Commands Tab**: Click "Send Heartbeat" to establish connection
2. **Command Long Tab**: Send emergency stop
   - Command ID: `400` (MAV_CMD_COMPONENT_ARM_DISARM)
   - Param1: `0` (Disarm)
   - Param2: `21196` (Force disarm magic number)
   - Click "Send Command Long"

**Expected Response**: All motors should stop immediately, system should report disarmed state

### 7. Debugging Custom RoboMaster Messages

**Scenario**: Send custom proprietary RoboMaster protocol messages

**Steps**:
1. **Custom Hex Tab**: Send custom RoboMaster packet
   ```
   AA 55 0C 00 FF 01 30 75 01 02 03 04 05 06 07 08 CRC1 CRC2
   ```
   - AA 55: RoboMaster frame header
   - 0C 00: Data length (12 bytes)
   - FF 01: Command type
   - 30 75: RoboMaster command ID
   - Payload: 01 02 03 04 05 06 07 08
2. **Monitor responses** for RoboMaster-specific acknowledgments

**Expected Response**: Custom response based on RoboMaster protocol implementation

### Troubleshooting RoboMaster Issues

**Common Problems**:
- **No motor response**: Check CAN bus wiring and termination resistors
- **Intermittent communication**: Verify baud rate and serial cable quality
- **Motors spinning incorrectly**: Check motor ID mapping and direction parameters
- **High latency**: Reduce message frequency or optimize MAVLink packet size

**Debugging Tips**:
- Use **received messages area** to monitor all incoming telemetry
- Check **connection status** indicator for communication health
- Send **heartbeat messages** regularly to maintain connection
- Use **raw hex mode** to send exact protocol packets for debugging
- Monitor **message counter** to verify bidirectional communication