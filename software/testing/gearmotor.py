import wiringpi
from time import sleep

MAX_SPEED = 480


class Motor:

    def __init__(self, pwm_pin, dir_pin):

        self.pwm_pin = pwm_pin
        self.dir_pin = dir_pin

        wiringpi.wiringPiSetupGpio()

        wiringpi.pinMode(pwm_pin, wiringpi.GPIO.PWM_OUTPUT)
        wiringpi.pwmSetMode(wiringpi.GPIO.PWM_MODE_MS)
        wiringpi.pwmSetRange(MAX_SPEED)
        wiringpi.pwmSetClock(2)

        wiringpi.pinMode(dir_pin, wiringpi.GPIO.OUTPUT)

    def set_speed(self, speed):

        if speed < 0:
            speed = -speed
            dir_value = 1
        else:
            dir_value = 0

        if speed > MAX_SPEED:
            speed = MAX_SPEED

        wiringpi.digitalWrite(self.dir_pin, dir_value)
        wiringpi.pwmWrite(self.pwm_pin, speed)


if __name__ == '__main__':
    motor = Motor(pwm_pin=12, dir_pin=5)
    motor.set_speed(380)
    sleep(10)
    motor.set_speed(-380)
    sleep(10)
    motor.set_speed(0)
