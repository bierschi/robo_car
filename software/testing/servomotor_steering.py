#!/usr/bin/env python3
from time import sleep
import Adafruit_PCA9685


class ServoMotor:

    def __init__(self, servo_min, servo_max, freq=60):

        self.pwm = Adafruit_PCA9685.PCA9685()

        self.servo_min = servo_min
        self.servo_max = servo_max

        self.freq = freq
        self.pwm.set_pwm_freq(self.freq)

    def move_extremas(self):

        while True:
            self.pwm.set_pwm(0, 0, self.servo_min)
            sleep(1)
            self.pwm.set_pwm(0, 0, self.servo_max)
            sleep(1)

    def drive_left(self):
        self.pwm.set_pwm(0, 0, self.servo_min)

    def drive_right(self):
        self.pwm.set_pwm(0, 0, self.servo_max)

    def drive_straight_ahead(self):
        self.pwm.set_pwm(0, 0, self.servo_max-self.servo_min)


def main():

    print("Servo Motor test: ")

    sm = ServoMotor(140, 490, 60)
    sm.drive_left()
    sleep(1)
    sm.drive_right()
    sleep(1)
    sm.drive_straight_ahead()


if __name__ == '__main__':
    main()
