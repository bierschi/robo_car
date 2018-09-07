#!/usr/bin/env python3
from time import sleep
import Adafruit_PCA9685


class ServoMotor:

    def __init__(self, servo_min, servo_max, channel, freq=60):

        self.pwm = Adafruit_PCA9685.PCA9685()

        self.servo_min = servo_min
        self.servo_max = servo_max

        if channel <= 15 and channel >= 0:
            self.channel = channel
        else:
            raise ValueError("select a channel between 0 and 15")

        if freq >= 40 and freq <=1000:
            self.freq = freq
        else:
            raise ValueError("select frequenz between 40hz and 1000hz")

        self.pwm.set_pwm_freq(self.freq)

    def move_extremas(self):

        while True:
            self.pwm.set_pwm(self.channel, 0, self.servo_min)
            sleep(1)
            self.pwm.set_pwm(self.channel, 0, self.servo_max)
            sleep(1)

    def drive_left(self):
        self.pwm.set_pwm(self.channel, 0, self.servo_min)

    def drive_right(self):
        self.pwm.set_pwm(self.channel, 0, self.servo_max)

    def drive_straight_ahead(self):
        self.pwm.set_pwm(self.channel, 0, self.servo_max-self.servo_min)


def main():

    print("Servo Motor test: ")

    sm_steering = ServoMotor(140, 490, channel=0, freq=60)
    sm_camera = ServoMotor(170,560, channel=15, freq=60)
    #sm_steering.move_extremas()
    #sm_camera.move_extremas()
    sleep(2)
    sm_steering.drive_left()
    sm_camera.drive_left()
    sleep(2)
    sm_steering.drive_right()
    sm_camera.drive_right()
    sleep(2)
    sm_steering.drive_straight_ahead()
    sm_camera.drive_straight_ahead()
    sleep(2)

if __name__ == '__main__':
    main()
