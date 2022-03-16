# SPDX-FileCopyrightText: 2020 Kattni Rembor for Adafruit Industries
#
# SPDX-License-Identifier: MIT
#
"""Sensor demo for Adafruit Feather Sense. Prints data from each of the sensors."""
import time
import array
import math
import board
import audiobusio
import adafruit_apds9960.apds9960
import adafruit_bmp280
import adafruit_lis3mdl
import adafruit_lsm6ds.lsm6ds33
import adafruit_sht31d
from adafruit_motorkit import MotorKit
from adafruit_motor import stepper
import neopixel
import digitalio
from adafruit_debouncer import Debouncer

i2c = board.I2C()
lsm6ds33 = adafruit_lsm6ds.lsm6ds33.LSM6DS33(i2c) # accel/gyro
motors = MotorKit(i2c=i2c)
motors_hot = False

drive_style = stepper.INTERLEAVE

pin = digitalio.DigitalInOut(board.SWITCH)
pin.direction = digitalio.Direction.INPUT
pin.pull = digitalio.Pull.UP

pixel = neopixel.NeoPixel(board.NEOPIXEL, 1, brightness=0.1)
pixel.fill(0xFFFF00)

k_p = 10
k_i = 40
k_d = 0.05

target_angle = 9
prev_angle = 0
error_sum = 0

def constrain(val, min_val, max_val):
    return min(max_val, max(min_val, val))

def drive_both_motors(steps=0):
    motor1dir = stepper.BACKWARD
    motor2dir = stepper.FORWARD

    if steps < 0:
        motor1dir = stepper.FORWARD
        motor2dir = stepper.BACKWARD
    
    for x in range(abs(int(steps))):
        motors.stepper1.onestep(direction=motor1dir, style=drive_style)
        motors.stepper2.onestep(direction=motor2dir, style=drive_style)

while True:
    # print("Acceleration: {:.2f} {:.2f} {:.2f} m/s^2".format(*lsm6ds33.acceleration))
    # print("Gyro: {:.2f} {:.2f} {:.2f} dps".format(*lsm6ds33.gyro))

    # start paste
    
    acc = lsm6ds33.acceleration
    gyro = lsm6ds33.gyro

    sample_time = 1

    # calculate the angle of inclination
    # acc_angle = math.atan2(acc[0], acc[2]) #*RAD_TO_DEG
    # gyro_rate = gyro[1]
    # gyro_angle = gyro_rate*sample_time;  
    # current_angle = 0.9934*(prev_angle + gyro_angle) + 0.0066*(acc_angle)
    
    # error = current_angle - target_angle
    # error_sum = error_sum + error;  
    # error_sum = constrain(error_sum, -300, 300)
    
    # #calculate output from P, I and D values
    # motor_power = k_p*(error) + k_i*(error_sum)*sample_time - k_d*(current_angle-prev_angle)/sample_time
    # prev_angle = current_angle

    # print("Angle: {:.2f} power: {:.2f} ".format(acc[0], motor_power))


    # end paste

    magnitude = -1
    motor_power = acc[2] * magnitude

    print("Angle: {:.2f} power: {:.2f} ".format(acc[2], motor_power))

    if not pin.value:
        if motors_hot:
            motors_hot = False
            pixel.fill(0xFFFF00)
            motors.stepper1.release()
            motors.stepper2.release()
        else:
            motors_hot = True
            pixel.fill(0x00FF00)
        time.sleep(0.5)

    if motors_hot:
        drive_both_motors(motor_power)
    
    # time.sleep(0.1)



