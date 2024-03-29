#SEED LAB DEMO 2
#Silje Ostrem

from smbus2 import SMBus
import time

ARD_i2c = SMBus(1)
ARD_ADDR = 0x08
FOV = 60

def write_data(angle,distance):
    angle_sent = (-angle+FOV/2)(255/FOV)
    angle_sent= round(angle_sent)
    ARD_i2c.write_byte_data(ARD_ADDR, angle_sent, distance)
