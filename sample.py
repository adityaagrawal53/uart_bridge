import serial
import time

ser = serial.Serial('/dev/ttyUSB0', 460800, timeout=1)

try:
    while True:
        # send data
        ser.write(b'Hello MCU\r\n')

        # read incoming data
        data = ser.readline()
        if data:
            print("RX:", data.decode(errors='ignore').strip())

except KeyboardInterrupt:
    print("Exiting...")

finally:
    ser.close()
