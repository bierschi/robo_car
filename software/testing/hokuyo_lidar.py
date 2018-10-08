from breezylidar import URG04LX


def main():
    laser = URG04LX('/dev/ttyACM0')

    while True:
        scan = laser.getScan()
        print(scan)


if __name__ == '__main__':
    main()
