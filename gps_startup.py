import time
import adafruit_gps
import serial
uart = serial.Serial("/dev/ttyUSB0", baudrate=9600, timeout=10)

# Create a GPS module instance.
gps = adafruit_gps.GPS(uart, debug=False)  # Use UART/pyserial

# Main loop runs forever printing the location, etc. every second.
last_print = time.monotonic()
while True:
    gps.update()
    if not gps.has_fix:
        print("Waiting for fix...")
        continue
    break
