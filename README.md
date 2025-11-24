# ESP32 MPU6886 IMU Sensor with ROS2 Integration

Complete IMU sensor system with ESP32 hardware and ROS2 software integration. Supports both wired (Serial) and wireless (WiFi/MQTT) communication modes.

## Hardware Setup

### Components
- ESP32 development board
- MPU6886 6-axis IMU sensor
- USB cable (for wired mode)
- WiFi network (for wireless mode)

### I2C Wiring

```
MPU6886    --    ESP32-32 / -C6
VCC        --    5V
GND        --    GND
SDA        --    GPIO21 (ESP32-32S / configurable)
SCL        --    GPIO22 (ESP32-32S / configurable)
```

## ESP32 Firmware

### Prerequisites

**Install ESP-IDF:**
```bash
# Install dependencies (Ubuntu/Debian)
sudo apt-get install git wget flex bison gperf python3 python3-pip python3-venv cmake ninja-build ccache libffi-dev libssl-dev dfu-util libusb-1.0-0

# Clone ESP-IDF
mkdir -p ~/esp
cd ~/esp
git clone --recursive https://github.com/espressif/esp-idf.git
cd esp-idf
git checkout v5.1  # or latest stable version

# Source environment (add to ~/.bashrc or ~/.zshrc for persistence)
. ~/esp/esp-idf/export.sh
```

**Quick ESP-IDF Setup (each terminal session):**
```bash
# Source ESP-IDF environment
. ~/esp/esp-idf/export.sh

# Or add alias to ~/.bashrc or ~/.zshrc:
# alias get_idf='. ~/esp/esp-idf/export.sh'
# Then just run: get_idf
```

### Features
- MPU6886 driver with temperature compensation
- Gyroscope + Accelerometer data
- CSV output format: `IMU,timestamp,ax,ay,az,gx,gy,gz,temp`
- Temperature calibration support
- Clean separation (header + implementation)

### Build & Flash

**Common ESP-IDF Commands:**
```bash
# Navigate to project
cd i2c_basic

# Full clean (removes build artifacts)
idf.py fullclean

# Configure project (optional, opens menuconfig)
idf.py menuconfig

# Build firmware
idf.py build

# Flash to ESP32 (replace /dev/ttyUSB1 with your port)
idf.py -p /dev/ttyUSB1 flash

# Monitor serial output
idf.py -p /dev/ttyUSB1 monitor

# Build, flash, and monitor in one command
idf.py -p /dev/ttyUSB1 flash monitor

# Erase flash completely (factory reset)
idf.py -p /dev/ttyUSB1 erase-flash

# Clean build (faster than fullclean)
idf.py clean
```

**Automated Build Script:**
Use the provided automation script (see [ESP32 Automation](#esp32-automation) section below):
```bash
./esp32_build.sh build        # Build only
./esp32_build.sh flash        # Build and flash
./esp32_build.sh monitor      # Flash and monitor
./esp32_build.sh clean        # Clean build
./esp32_build.sh fullclean    # Full clean
```

### Output Format
The ESP32 outputs CSV data over serial:
```
IMU,<timestamp_ms>,<ax>,<ay>,<az>,<gx>,<gy>,<gz>,<temp>
```
- Acceleration in m/s²
- Gyroscope in rad/s
- Temperature in °C

## ROS2 Package

### Dependencies
```bash
sudo apt install ros-humble-desktop
sudo apt install libsqlite3-dev
```

### Build
```bash
cd ros2_ws
colcon build --packages-select imu_sensor_pkg
source install/setup.bash
```

### Nodes

#### 1. Serial IMU Lifecycle Node
**Purpose:** Reads IMU data from ESP32 via USB serial

**Features:**
- Full ROS2 Lifecycle management
- Configurable serial port and baud rate
- Publishes to `/imu/data` (sensor_msgs/Imu)
- Publishes to `/imu/temperature` (sensor_msgs/Temperature)

**Parameters:**
- `serial_port`: Serial device path (default: `/dev/ttyUSB1`)
- `baud_rate`: Serial baud rate (default: `115200`)
- `publish_rate_ms`: Publishing rate in ms (default: `100`)

#### 2. Database Subscriber Node
**Purpose:** Stores IMU data to SQLite database

**Features:**
- Automatic database creation
- Stores timestamped IMU + temperature data
- Schema: `id, timestamp, accel(xyz), gyro(xyz), temperature`

**Parameters:**
- `database_path`: SQLite database file (default: `imu_data.db`)

## Quick Start

### Serial Mode
See [SERIAL_MODE_QUICKSTART.md](SERIAL_MODE_QUICKSTART.md) for complete setup instructions.

**Quick version:**
```bash
# 1. Flash ESP32
cd i2c_basic
<<<<<<< Updated upstream
idf.py -p /dev/ttyUSB0 flash
=======
idf.py -p /dev/ttyUSB1 flash
```
>>>>>>> Stashed changes

# 2. Launch ROS2
cd ros2_ws
source install/setup.bash
ros2 launch imu_sensor_pkg imu_serial.launch.py
```

### Wireless Mode (WiFi/MQTT)
See [WIRELESS_MODE_QUICKSTART.md](WIRELESS_MODE_QUICKSTART.md) for complete setup instructions.

**Quick version:**
```bash
# 1. Flash ESP32 (creates WiFi AP: g1-imu6866)
cd i2c_basic
idf.py -p /dev/ttyUSB0 flash

# 2. Connect laptop to ESP32 WiFi
# SSID: g1-imu6866
# Password: 69420imu

# 3. Start Mosquitto
sudo systemctl start mosquitto

# 4. Launch ROS2
cd ros2_ws
source install/setup.bash
ros2 launch imu_sensor_pkg imu_mqtt.launch.py
```

## File Structure

```
.
├── i2c_basic/                     # ESP32 firmware
│   ├── main/
│   │   ├── mpu6886_main.h         # MPU6886 driver header
│   │   ├── mpu6886_main.cpp       # Main implementation
│   │   └── CMakeLists.txt
│   ├── build/
│   └── sdkconfig
├── ros2_ws/                        # ROS2 workspace
│   └── src/
│       └── imu_sensor_pkg/         # ROS2 package
│           ├── src/
│           │   ├── serial_imu_lifecycle_node.cpp
│           │   └── imu_database_subscriber.cpp
│           ├── launch/
│           │   └── imu_serial.launch.py
│           ├── CMakeLists.txt
│           └── package.xml
└── README.md                       # This file
```

## Troubleshooting

### Serial Port Permission
```bash
sudo usermod -a -G dialout $USER
# Logout and login again
```

### Find Serial Port
```bash
ls /dev/ttyUSB*
# or
dmesg | grep tty
```

### Check IMU Data
```bash
# Direct serial monitor
screen /dev/ttyUSB1 115200
# or
idf.py -p /dev/ttyUSB1 monitor
```

### Database Verification

#### Quick Database Queries
```bash
# 1) List all tables
sqlite3 ros2_ws/imu_data.db ".tables"

# 2) Show table schema
sqlite3 ros2_ws/imu_data.db "SELECT sql FROM sqlite_master WHERE type='table' AND name='imu_data';"

# 3) Show 10 most recent entries (with headers, CSV format)
sqlite3 -header -csv ros2_ws/imu_data.db "SELECT * FROM imu_data ORDER BY id DESC LIMIT 10;"

# 4) Export entire database to CSV
sqlite3 -header -csv ros2_ws/imu_data.db "SELECT * FROM imu_data;" > ~/imu_data_export.csv

# 5) Count total entries
sqlite3 ros2_ws/imu_data.db "SELECT COUNT(*) FROM imu_data;"

# 6) Show data in specific time range
sqlite3 -header -csv ros2_ws/imu_data.db "SELECT * FROM imu_data WHERE timestamp_sec BETWEEN 1000 AND 2000 LIMIT 20;"
```

#### Interactive Database Shell
```bash
sqlite3 ros2_ws/imu_data.db
> .schema
> SELECT COUNT(*) FROM imu_data;
> SELECT * FROM imu_data ORDER BY id DESC LIMIT 5;
> .quit
```

## Temperature Calibration

The ESP32 supports one-point temperature calibration:

1. **Measure actual temperature** with a reference thermometer
2. **Start ESP32** and at the prompt type:
   ```
   cal 21.5
   ```
   (replace 21.5 with your measured temperature)
3. **Note the suggested offset** from logs
4. **(Optional)** Hard-code the offset in `mpu6886_main.cpp`:
   ```cpp
   static float g_mpu6886_temp_offset = <suggested_value>;
   ```

## Development

### ESP32 Automation

A simple shell script (`esp32_build.sh`) is provided to automate ESP-IDF operations. The script automatically sources ESP-IDF if needed.

**Usage:**
```bash
# Make script executable (first time only)
chmod +x esp32_build.sh

# Build firmware
./esp32_build.sh build

# Flash to ESP32 (default port: /dev/ttyUSB1)
./esp32_build.sh flash

# Flash with custom port
./esp32_build.sh flash /dev/ttyUSB0

# Flash and monitor serial output
./esp32_build.sh monitor

# Flash and monitor with custom port
./esp32_build.sh monitor /dev/ttyUSB0

# Clean build artifacts
./esp32_build.sh clean

# Full clean (removes all build files)
./esp32_build.sh fullclean
```

**Note:** The script automatically sources ESP-IDF from `~/esp/esp-idf/export.sh` if not already sourced.

### Extending Database Schema
Modify `imu_database_subscriber.cpp` table creation SQL and insert statements.

## License

MIT

## Authors

- ESP32 Firmware: MPU6886 driver with WiFi/MQTT support
- ROS2 Integration: Lifecycle node architecture with SQLite persistence
# g1_25_assignment4_odometry_ros2_ws-Public
