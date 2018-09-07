#!/usr/bin/env python3
from time import sleep
import Adafruit_PCA9685


class ServoMotor:

    def __init__(self, servo_min, servo_max, freq=60):

        self.pwm = Adafruit_PCA9685.PCA9685()

        self.servo_min = servo_min
        self.servo_max = servo_max

        self.freq = 60
        self.pwm.set_pwm_freq(self.freq)

    def move_extremas(self):

        while True:
            self.pwm.set_pwm(0, 0, self.servo_min)
            sleep(1)
            self.pwm.set_pwm(0, 0, self.servo_max)
            sleep(1)


def main():

    print("Servo Motor test: ")

    sm = ServoMotor(140, 490, 60)
    sm.move_extremas()


if __name__ == '__main__':
    main()
