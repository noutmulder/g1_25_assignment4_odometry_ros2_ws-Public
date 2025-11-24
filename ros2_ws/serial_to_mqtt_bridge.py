#!/usr/bin/env python3
"""
Serial to MQTT Bridge - Reads ESP32 serial data and publishes to MQTT
This allows you to see real ESP32 sensor data in MQTTX while connected via USB
"""

import serial
import paho.mqtt.client as mqtt
import time
import sys

# Configuration
SERIAL_PORT = "/dev/ttyUSB1"
BAUD_RATE = 115200
MQTT_BROKER = "localhost"
MQTT_PORT = 1883
MQTT_TOPIC = "esp32/imu/data"

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print(f"[OK] Connected to MQTT broker {MQTT_BROKER}:{MQTT_PORT}")
        print(f"[OK] Publishing ESP32 serial data to topic: {MQTT_TOPIC}")
        print("-" * 60)
    else:
        print(f"[ERROR] Failed to connect to MQTT broker, return code {rc}")

def main():
    # Setup MQTT client
    mqtt_client = mqtt.Client()
    mqtt_client.on_connect = on_connect
    
    try:
        print(f"Connecting to MQTT broker {MQTT_BROKER}:{MQTT_PORT}...")
        mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
        mqtt_client.loop_start()
        
        # Open serial port
        print(f"Opening serial port {SERIAL_PORT} at {BAUD_RATE} baud...")
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        time.sleep(2)  # Wait for serial to stabilize
        
        print("[OK] Serial port opened")
        print("[OK] Reading ESP32 data and publishing to MQTT...")
        print("[OK] Press Ctrl+C to stop")
        print("-" * 60)
        
        msg_count = 0
        
        while True:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                
                # Look for IMU data lines (format: IMU,timestamp,ax,ay,az,gx,gy,gz,temp)
                if line.startswith("IMU,"):
                    # Publish to MQTT
                    mqtt_client.publish(MQTT_TOPIC, line)
                    msg_count += 1
                    
                    # Print every 10th message
                    if msg_count % 10 == 0:
                        print(f"[{msg_count:4d}] Published: {line}")
                    
                    # Also print the formatted log line if it exists
                elif "mpu6886" in line and "Accel:" in line:
                    # This is the human-readable log line, just print it
                    pass  # We only need the CSV data for MQTT
            
            time.sleep(0.01)  # Small delay to prevent CPU hogging
                    
    except serial.SerialException as e:
        print(f"\n[ERROR] Serial port error: {e}")
        print(f"  Make sure ESP32 is connected to {SERIAL_PORT}")
        print(f"  Check with: ls -l /dev/ttyUSB*")
        sys.exit(1)
        
    except KeyboardInterrupt:
        print("\n\nStopping Serial-to-MQTT bridge...")
        ser.close()
        mqtt_client.loop_stop()
        mqtt_client.disconnect()
        print(f"Total messages published: {msg_count}")
        
    except Exception as e:
        print(f"\n[ERROR] Error: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()
