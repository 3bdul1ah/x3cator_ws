#!/usr/bin/env python3
import serial
import time
import ctypes
from dataclasses import dataclass
from datetime import datetime

@dataclass
class GasSensorData:
    timestamp: datetime
    CO: float
    H2S: float
    O2: float
    CH4: float

class FourGasSensor:
    def __init__(self):
        try:
            self.sensor = serial.Serial(
                port='/dev/ttyUSB1',
                baudrate=9600,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.5
            )
        except serial.SerialException as e:
            if 'Permission denied' in str(e):
                print("Permission denied accessing /dev/ttyUSB*.")
                print("Please run with sudo.")
            raise

    def read_sensor(self) -> GasSensorData:
        try:
            while self.sensor.read(1) != b'\xff':
                continue
            while self.sensor.read(1) != b'\x86':
                continue

            inBytes = [0xFF, 0x86]
            inBytes.extend(self.sensor.read(9))

            checksumByte = inBytes.pop()
            checksum = ctypes.c_ubyte(~((sum(inBytes)) % 256)).value

            if checksum == checksumByte:
                CO = int.from_bytes([inBytes[2], inBytes[3]], 'big', signed=False) * 1.00
                H2S = int.from_bytes([inBytes[4], inBytes[5]], 'big', signed=False) * 1.00
                O2 = int.from_bytes([inBytes[6], inBytes[7]], 'big', signed=False) * 0.1
                CH4 = int.from_bytes([inBytes[8], inBytes[9]], 'big', signed=False) * 1.00

                CO = max(0, CO)
                H2S = max(0, H2S)
                CH4 = CH4 - 100.0

                return GasSensorData(
                    timestamp=datetime.now(),
                    CO=CO,
                    H2S=H2S,
                    O2=O2,
                    CH4=CH4
                )
            else:
                raise ValueError("Checksum verification failed")

        except serial.SerialException as e:
            print(f"Error reading from sensor: {e}")
            raise

    def close(self):
        """Close the serial connection."""
        if self.sensor.is_open:
            self.sensor.close()

def main():
    try:
        sensor = FourGasSensor()
        print("Four gas sensor started")

        while True:
            try:
                data = sensor.read_sensor()
                print(f"Timestamp: {data.timestamp}")
                print(f"CO: {data.CO:.2f} ppm, H2S: {data.H2S:.2f} ppm, "
                      f"O2: {data.O2:.2f} %, CH4: {data.CH4:.2f} %LEL")
                time.sleep(0.2)

            except KeyboardInterrupt:
                print("\nStopping sensor readings...")
                break

            except Exception as e:
                print(f"Error: {e}")
                time.sleep(1)
                continue

    finally:
        if 'sensor' in locals():
            sensor.close()

if __name__ == '__main__':
    main()