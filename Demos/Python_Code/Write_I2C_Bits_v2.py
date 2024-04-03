#SEED LAB DEMO 2
#Silje Ostrem, Ben Sprik

from smbus2 import SMBus
import time

ARD_i2c = SMBus(1)
ARD_ADDR = 0x08
FOV = 60

def write_data(angle,distance):
    angle_sent = round((-angle+FOV/2)(255/FOV))
    angle_high = angle_sent >> 8
    angle_low = angle_sent & 0xFF
    distance_high = distance >> 8
    distance_low = distance & 0xFF
    data = [angle_high, angle_low, distance_high, distance_low]
    ARD_i2c.write_i2c_block_data(ARD_ADDR, 0, data)
