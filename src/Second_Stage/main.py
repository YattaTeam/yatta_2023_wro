import serial
from mpu_ctrl import MPU
from encoder import Encoder
from gpiozero import Button
from motor_ctrl import mctrl
from time import sleep, time
from threading import Thread, Event
from color_handler import ColorsCoordinations

import numpy as np
from typing import List


def read_distance(event: Event):
    global frontDist, rightDist, leftDist
    ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
    ser.reset_input_buffer()

    while not event.is_set():
        if ser.in_waiting > 0:
            frontDist, rightDist, leftDist = list(
                map(float, ser.readline().decode("utf-8").rstrip().split(",")))
            print(
                f"\nFRONT:: {frontDist}\nRIGHT:: {rightDist}\nLEFT:: {leftDist}\n")

            motors.update_m_dist([frontDist, rightDist, leftDist])

            sleep(0.004)


def loop(mpu: MPU):
    mpu.setUp()
    mpu.calibrateGyro(500)

    while True:
        mpu.currentAngle = mpu.compFilter(1)


def get_color(event: Event):
    orange_vals = {
        "lower": np.array([4, 50, 0], np.uint8),
        "upper": np.array([26, 255, 255], np.uint8)
    }

    blue_vals = {
        "lower": np.array([90, 105, 90], np.uint8),
        "upper": np.array([125, 255, 255], np.uint8)
    }

    green_vals = {
        "lower": np.array([50, 87, 65], np.uint8),
        "upper": np.array([79, 255, 255], np.uint8)
    }

    red1 = {
        "lower": np.array([0, 61, 84], np.uint8),
        "upper": np.array([10, 255, 255], np.uint8)
    }

    red2 = {
        "lower": np.array([163, 61, 80], np.uint8),
        "upper": np.array([170, 255, 255], np.uint8)
    }

    motors.direction = 0

    while not event.is_set():

        if motors.direction == 0:
            orange = color_reco.get_coords(
                orange_vals["lower"], orange_vals["upper"])
            blue = color_reco.get_coords(blue_vals["lower"], blue_vals["upper"])

            if orange != (-1, -1):
                motors.direction = -1
            elif blue != (-1, -1):
                motors.direction = 1
            else:
                if leftDist - rightDist > 82:
                    motors.direction = 1
                elif rightDist - leftDist > 82:
                    motors.direction = -1

        red1_area = color_reco.get_area(red1["lower"], red1["upper"])
        red2_area = color_reco.get_area(red2["lower"], red2["upper"])
        green_area = color_reco.get_area(green_vals["lower"], green_vals["upper"])

        if red1_area > 0 or red2_area > 0:
            motors.cube_adjust = 20
        elif green_area > 0:
            motors.cube_adjust = -20


def starting_point_adjustment():
    heading = mpu.currentAngle
    motors.move_forward(1)

    while frontDist > 130:
        motors.adjust_angle(heading, mpu.currentAngle)

    print("out")

    motors.stop_car()
    motors.turn_forward()


def moveUntilDist():
    heading = mpu.currentAngle
    motors.move_forward()

    motors.minfo(f"\n\nFRONT:: {frontDist}\n\n")
    while (frontDist > 80 or frontDist == 0) or (leftDist < 140 and rightDist < 140) and frontDist > 55:

        # if frontDist < 50:
        #     motors.move_forward(motors.speed - 0.005)
        motors.adjust_angle(heading, mpu.currentAngle)

        motors.minfo(
            f"\nFRONT:: {frontDist}\nRIGHT:: {rightDist}\nLEFT:: {leftDist}\n")
        sleep(0.0004)

    print("OUT OF LOOP")
    motors.stop_car()
    motors.turn_forward()


def turn():
    heading = mpu.currentAngle

    mpu.nextAngle = motors.get_next_angle(heading)
    motors.direction_turn()
    motors.move_forward(1)

    mpu.dinfo(
        f"ABOVE:: ({mpu.currentAngle} - {mpu.nextAngle}) * {motors.direction} = {(mpu.nextAngle - mpu.currentAngle) * motors.direction}")

    while not (250 > (mpu.currentAngle - mpu.nextAngle) * motors.direction >= -5):
        mpu.dinfo(
            f"({mpu.currentAngle} - {mpu.nextAngle}) * {motors.direction} = {(mpu.nextAngle - mpu.currentAngle) * motors.direction}")

    mpu.dinfo(
        f"UNDER:: ({mpu.currentAngle} - {mpu.nextAngle}) * {motors.direction} = {(mpu.nextAngle - mpu.currentAngle) * motors.direction}")

    motors.stop_car()
    motors.turn_forward()


def safe_close(events: List[Event]):
    for event in events:
        event.set()


def main():

    motors.turn_forward()

    color_event = Event()
    dist_event = Event()

    calThread = Thread(target=loop, args=(mpu)).start()

    ready = mpu.isReady()

    while not btn.is_active:
        print("waiting for button")
        sleep(0.2)

    colorThread = Thread(target=get_color, args=(color_event, )).start()
    distThread = Thread(target=read_distance, args=(dist_event, )).start()

    motors.max_angle = 120
    motors.min_angle = 50

    # Thread safety
    sleep(0.1)
    for i in range(3):
        if i == 0:
            motors.speed = 0.7
        for j in range(4):
            moveUntilDist()
            motors.speed = 1
            sleep(0.05)
            turn()
            sleep(0.05)

        print(f"Turn {i + 1}")

    print("Finishing")
    starting_point_adjustment()
    safe_close([color_event, dist_event])


if __name__ == '__main__':

    frontDist = 0
    rightDist = 0
    leftDist = 0

    debugList = [False, False]

    motors = mctrl(motorPins=[24, 13], servoPin=12,
                   speed=1, debug=debugList[0])
    mpu = MPU(gyro=250, acc=2, tau=0.98, debug=debugList[1])

    color_reco = ColorsCoordinations(60, "auto")
    btn = Button(20)

    main()
