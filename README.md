# RoboCar
The main aim of this repo is to set up a low-cost remote-controlled robotic car
and later on a completely autonomous driving car with a Raspberry Pi 3b+

#### Table of Contents:

- [Hardware](https://github.com/bierschi/robo_car#hardware)
    - [Components]()
    - [Circuit Diagram]()
- [Software](https://github.com/bierschi/robo_car#software)
    - [Remote-controlled](https://github.com/bierschi/robo_car#remote-controlled)
    - [Autonomous driving](https://github.com/bierschi/robo_car#autonomous-driving)
- [Operating System](https://github.com/bierschi/robo_car#operating-system)
- [Project Layout](https://github.com/bierschi/robo_car#project-layout)


## Hardware
To build this platform, two essential part have to be considered. On
the one hand the used hardware components and on the other hand the wiring of
the individual components

#### Components
Here are listed all hardware components that were used to build this RoboCar

- 4WD RC Smart Auto Chassis: [Auto Chassis](https://de.aliexpress.com/item/4WD-RC-Smart-Auto-Chassis-F-r-Arduino-Plattform-Mit-MG996R-Metal-Gear-Servo-Lagersatz-Lenkgetriebe/32830665408.html?spm=a2g0x.search0104.3.2.106a2f5f4Hjmhg&ws_ab_test=searchweb0_0%2Csearchweb201602_4_10320_10152_10065_10151_10344_10068_10342_10547_10343_10340_10341_10548_10696_10084_10083_10618_10304_5725020_10307_10820_10821_10302_5724920_5724120_10843_5724020_10059_100031_10319_5724320_10103_10624_10623_10622_10621_10620_5724220%2Csearchweb201603_2%2CppcSwitch_5&algo_expid=0a82566b-deae-43f9-9fa3-5f2403602186-0&algo_pvid=0a82566b-deae-43f9-9fa3-5f2403602186&transAbTest=ae803_2&priceBeautifyAB=0)
- Servomotor MG996R (already included in the auto chassis): [Servomotor](https://www.ebay.de/itm/192047974387)
- Gearmotor (already included in the auto chassis): [Gearmotor](https://www.ebay.de/itm/132733015168)
- Motordriver Pololu DRV8838: [Motor Driver](https://www.ebay.de/itm/Pololu-DRV8838-Single-Brushed-DC-Motor-Driver-Carrier-2990/272351389574?ssPageName=STRK%3AMEBIDX%3AIT&_trksid=p2057872.m2749.l2649)
- Raspberry Pi 3b+: [Raspberry Pi 3b+](https://www.amazon.de/dp/B07BDR5PDW/ref=sxnav_sxwds-bovbp-i_m_2?pf_rd_m=A3JWKAKR8XB7XF&pf_rd_p=b3231e2b-a779-4655-bc87-f09acb903eca&pd_rd_wg=0coGd&pf_rd_r=GXQJP37HWMA1E86RFGCP&pf_rd_s=desktop-sx-nav&pf_rd_t=301&pd_rd_i=B07BDR5PDW&pd_rd_w=zJXAB&pf_rd_i=raspberry+pi+3+b%2B&pd_rd_r=063492d1-ecbd-4f0a-b68a-2442561c6d08&ie=UTF8&qid=1535019097&sr=2)
- Ultrasonic ranging module HC-SR04 + SG90 Servomotor: [Ultrasonic + Servo](https://www.ebay.de/itm/SG90-Servo-HC-SR04-Ultraschall-Entfernungsmodul-KFZ-Halterung/322711484715?ssPageName=STRK%3AMEBIDX%3AIT&var=511769911508&_trksid=p2060353.m2749.l2649)
- PCA9685 PWM/Servo Driver: [PCA9685](https://www.ebay.de/itm/PCA9685-16-Kanal-Driver-Servomotor-Treiber-Modul-PWM-I2C-Arduino-Raspberry-Pi/253285067342?ssPageName=STRK%3AMEBIDX%3AIT&_trksid=p2057872.m2749.l2649)
- Laser Scanner Hokuyo URG-04LX-UG01: [Hokuyo](https://www.robotshop.com/de/de/hokuyo-urg-04lx-ug01-scan-laser-entfernungsmesser-eu.html?gclid=Cj0KCQjw6MHdBRCtARIsAEigMxFSq916Qq001TadjiVhvlQdLHBiCjHoI453lspePQd77kCsp5AZhw4aApSKEALw_wcB)

Datasheets for every component can be found under `/hardware`

#### Circuit Diagram
The Circuit Diagram is shown below to connect the individual components.
A pdf file is under `/circuit_diagram`

<div align="center">
  <br>
  <img src="images/circuit_diagram_robocar.png" alt="example" width="584" height="380">
</div>

## Software

This Software is written in C++, build with [CMake](https://cmake.org/) and is divided in
`Remote-controlled` and `Autonomous driving` with different submodules:

### Remote-controlled:
##### RoboCar:

##### GUI:

##### Testing:

### Autonomous driving:


## Operating System


## Project Layout
<pre><code>
/hardware
    /chassis
    /motors
    /power_supply
    /sensors
    README.md

/pictures

/scripts
    README.md
    os_settings.sh

/software
    /gui
        /build
        /images
        /include
            ClientSocket.h
            Socket.h
            SocketException.h
            mainwindow.h
        /src
            ClientSocket.cpp
            Socket.cpp
            mainwindow.cpp
            mainwindow.ui
        main_gui.cpp
        CMakeLists.txt
    /robocar
        /build
        /configs
        /include
            /comm
                I2C.h
                ServerSocket.h
                Socket.h
                SocketException.h
            /sensors
                Camera.h
                GearMotor.h
                MPU6050.h
                PCA9685.h
                Ultrasonic.h
        /lib
        /src
            /comm
                I2C.cpp
                ServerSocket.cpp
                Socket.cpp
            /sensors
                Camera.cpp
                GearMotor.cpp
                MPU6050.cpp
                PCA9685.cpp
                Ultrasonic.cpp
        CMakeLists.txt
        robocar_main.cpp
    /testing
        client.cpp
        client.h
        gearmotor.py
        main_client.cpp
        main_i2c.cpp
        main_server.cpp
        main_serversocket.cpp
        picamera_stream.py
        server.cpp
        server.h
        servomotor.py
        ultrasonic.py
    README.md
LICENSE
README.md

</pre></code>