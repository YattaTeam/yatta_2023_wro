from gpiozero import Motor, AngularServo
from typing import Union
from time import sleep


class mctrl():
    def __init__(self, motorPins: list, servoPin: int, speed: int = 1, debug: bool = False) -> None:
        self.motorPins: list = motorPins
        self.servoPin: int = servoPin
        self.motor: Motor = Motor(*self.motorPins)
        self.servo: AngularServo = AngularServo(
            self.servoPin, min_angle=0, max_angle=180, min_pulse_width=0.0005, max_pulse_width=0.0025)

        self.debug = debug

        self.__speed = speed
        self.__min_angle = 45
        self.__max_angle = 135
        self.__mid_angle = 90
        self.__direction = 0

        self.cube_adjust = 0

        self.front_distance, self.left_distance, self.right_distance = 0, 0, 0

    @property
    def direction(self):
        return self.__direction

    @direction.setter
    def direction(self, val):
        if val not in [1, 0, -1]:
            self.__direction = 0
            print("Direction should be in [1, 0, -1] ... Resseting to 0")
        self.__direction = val

    @property
    def mid_angle(self):
        return self.__mid_angle

    @mid_angle.setter
    def mid_angle(self, val):
        self.__mid_angle = val

    @property
    def min_angle(self):
        return self.__min_angle

    @min_angle.setter
    def min_angle(self, val):
        self.__min_angle = val

    @property
    def max_angle(self):
        return self.__max_angle

    @max_angle.setter
    def max_angle(self, val):
        self.__max_angle = val

    @property
    def speed(self):
        return self.__speed

    @speed.setter
    def speed(self, val):
        self.__speed = min(1, max(0, val))

    def minfo(self, val):
        if self.debug:
            print(val)

    def update_m_dist(self, updated: list):
        self.front_distance, self.left_distance, self.right_distance = updated

    def get_next_angle(self, angle: Union[int, float]):
        return ((angle + 5) + (self.__direction * 90)) % 360
        # return ((angle + 5) + (self.__direction * 90)) % 360 // 90 * 90

    def map_angle(self, angle: Union[int, float]):
        return max(self.min_angle, min(self.max_angle, angle))

    def adjust_angle(self, heading: Union[int, float], reading: Union[int, float]):

        distance_error = 0

        if 0 < self.right_distance < 17.567:
            distance_error = -9
        elif 0 < self.left_distance < 17.567:
            distance_error = 15

        error_value = reading - heading + distance_error
        error_value = ((error_value + 180) % 360 - 180)

        adjustment = error_value + 85 + self.cube_adjust

        self.servo.angle = self.map_angle(adjustment)

    def direction_turn(self):
        if self.direction == -1:
            self.turn_right()
        elif self.direction == 1:
            self.turn_left()

    def fix_forward(self):
        self.stop_car()

        if self.left_distance > self.right_distance:
            self.turn_right()
        elif self.right_distance > self.left_distance:
            self.turn_left()

        self.move_backward(speed=1)
        sleep(0.5)
        self.move_forward(self.speed)

    def fix_stuck(self, heading: Union[float, int], reading: Union[float, int]):
        self.reverse_angle()
        self.move_backward(speed=1)
        sleep(0.505)
        self.stop_car()
        self.adjust_angle(heading, reading)
        self.move_forward(self.speed)

    def reverse_angle(self):
        newAngle = (90 - self.servo.angle) + 90
        self.servo.angle = self.map_angle(newAngle)

    def turn_forward(self):
        self.servo.angle = self.mid_angle

    def turn_left(self):
        self.servo.angle = self.min_angle

    def turn_right(self):
        self.servo.angle = self.max_angle

    def move_forward(self, speed: int = 1):
        self.motor.forward(self.speed)

    def move_backward(self, speed: int = 1):
        self.motor.backward(speed)

    def stop_car(self):
        self.motor.stop()
