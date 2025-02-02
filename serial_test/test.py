import sys
import json
import time
import serial
import random
import string
from datetime import datetime
import argparse

class TelemetryTest:
    def __init__(self, port, baudrate=9600, telem_interval=0.5, string_interval=2.0):
        self.ser = serial.Serial(port, baudrate, timeout=0)
        self.telem_interval = telem_interval
        self.string_interval = string_interval
        self.last_telem_time = time.time()
        self.last_string_time = time.time()
        
        # Initialize random data ranges
        self.euler_range = (-180.0, 180.0)
        self.velocity_range = (-100.0, 100.0)
        self.input_range = (-1.0, 1.0)

    def generate_telem_data(self):
        """Generate randomized telemetry data"""
        return {
            "data_type": "telem",
            "payload": {
                "euler_x": 
                    random.uniform(*self.euler_range),
                "euler_y": 
                    random.uniform(*self.euler_range),
                "euler_z":
                    random.uniform(*self.euler_range),
                "input_x":
                    random.uniform(*self.input_range),
                "input_y":
                    random.uniform(*self.input_range),
                "velocity_x":
                    random.uniform(*self.velocity_range),
                "velocity_y":
                    random.uniform(*self.velocity_range),
                "velocity_z":
                    random.uniform(*self.velocity_range),
                "dt": 0.0,
                "timestamp": datetime.now().isoformat()
            }
        }

    def generate_random_string(self):
        """Generate a random string message"""
        length = random.randint(5, 15)
        characters = string.ascii_lowercase + string.digits
        return {
            "data_type": "string",
            "payload": ''.join(random.choices(characters, k=length))
        }

    def non_blocking_read(self):
        """Windows-compatible non-blocking read"""
        try:
            while self.ser.in_waiting > 0:
                line = self.ser.readline().decode().strip()
                if line:
                    print(f"Received: {line}")
        except UnicodeDecodeError:
            print("Received invalid data")
        except Exception as e:
            print(f"Read error: {str(e)}")

    def run(self):
        try:
            print(f"Starting telemetry test on {self.ser.port}...")
            print(f"Telemetry interval: {self.telem_interval}s, String interval: {self.string_interval}s")
            
            while True:
                current_time = time.time()
                
                # Send telemetry data
                if current_time - self.last_telem_time >= self.telem_interval:
                    telem_data = self.generate_telem_data()
                    json_data = json.dumps(telem_data)
                    self.ser.write((json_data + '\n').encode())
                    print(f"Sent Telemetry: {json_data}")
                    self.last_telem_time = current_time
                
                # Send random strings
                if current_time - self.last_string_time >= self.string_interval:
                    string_data = self.generate_random_string()
                    json_data = json.dumps(string_data)
                    self.ser.write((json_data + '\n').encode())
                    print(f"Sent String: {json_data}")
                    self.last_string_time = current_time
                
                # Check for incoming data
                self.non_blocking_read()
                
                time.sleep(0.01)  # Reduce CPU usage

        except KeyboardInterrupt:
            print("\nTest stopped by user")
        finally:
            self.ser.close()
            print("Serial port closed")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Telemetry Test Script')
    parser.add_argument('port', help='Serial port name')
    parser.add_argument('--baud', type=int, default=115200, help='Baud rate')
    parser.add_argument('--telem-interval', type=float, default=0.5,
                       help='Telemetry send interval in seconds')
    parser.add_argument('--string-interval', type=float, default=2.0,
                       help='Random string send interval in seconds')
    
    args = parser.parse_args()
    
    test = TelemetryTest(
        port=args.port,
        baudrate=args.baud,
        telem_interval=args.telem_interval,
        string_interval=args.string_interval
    )
    
    test.run()