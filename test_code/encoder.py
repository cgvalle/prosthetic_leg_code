import os
from smbus2 import SMBus
from time import sleep
DEVICE_AS5600 = 0x36 # Default device I2C address
bus = SMBus(1)
def ReadRawAngle(): # Read angle (0-360 represented as 0-4096)
    read_bytes = bus.read_i2c_block_data(DEVICE_AS5600, 0x0C, 2)
    angle = (read_bytes[0]<<8) | read_bytes[1];
    # to degress
    angle = angle * 360 / 4096
    return angle


while True:
    os.system('clear')
    print(ReadRawAngle())
    sleep(0.1)
