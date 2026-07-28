import os
from smbus2 import SMBus
from time import sleep

DEVICE_AS5600 = 0x36 # Default device I2C address
bus = SMBus(1)

ESTIRADA = 230
CONTRAIDA = 170
def clamp(x, lo=0.0, hi=1.0):
    return lo if x < lo else hi if x > hi else x

def normalizar(valor, cero, uno):
    return clamp((valor - cero) / (uno - cero))

def ReadRawAngle(): # Read angle (0-360 represented as 0-4096)
    read_bytes = bus.read_i2c_block_data(DEVICE_AS5600, 0x0C, 2)
    angle = (read_bytes[0]<<8) | read_bytes[1];
    # to degress
    angle = angle * 360 / 4096

    # goes from 0 to 90 knee flexion. 0 is fully extended
    angle = normalizar(angle, ESTIRADA, CONTRAIDA) * 90
    return angle


while True:
    os.system('clear')
    print(ReadRawAngle())
    sleep(0.1)
