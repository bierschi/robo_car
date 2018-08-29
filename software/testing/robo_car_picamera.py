#!/usr/bin/env python3
from picamera import PiCamera


def main():
    camera = picamera.PiCamera()
    camera.resolution = (640, 480)
    camera.start_preview()


if __name__ == '__main__':
    main()
