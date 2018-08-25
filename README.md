# RoboCar
Build a remote-controlled robot car with a Raspberry Pi 3b+

- [Hardware](https://github.com/bierschi/robo_car#hardware)
- [Software](https://github.com/bierschi/robo_car#software)
- [Operating System](https://github.com/bierschi/robo_car#operating-system)
- [Project Layout](https://github.com/bierschi/robo_car#project-layout)


## Hardware

- 4WD RC Smart Auto Chassis: [Auto Chassis](https://de.aliexpress.com/item/4WD-RC-Smart-Auto-Chassis-F-r-Arduino-Plattform-Mit-MG996R-Metal-Gear-Servo-Lagersatz-Lenkgetriebe/32830665408.html?spm=a2g0x.search0104.3.2.106a2f5f4Hjmhg&ws_ab_test=searchweb0_0%2Csearchweb201602_4_10320_10152_10065_10151_10344_10068_10342_10547_10343_10340_10341_10548_10696_10084_10083_10618_10304_5725020_10307_10820_10821_10302_5724920_5724120_10843_5724020_10059_100031_10319_5724320_10103_10624_10623_10622_10621_10620_5724220%2Csearchweb201603_2%2CppcSwitch_5&algo_expid=0a82566b-deae-43f9-9fa3-5f2403602186-0&algo_pvid=0a82566b-deae-43f9-9fa3-5f2403602186&transAbTest=ae803_2&priceBeautifyAB=0)
- Servomotor MG996R (already included in the auto chassis): [Servomotor](https://www.ebay.de/itm/192047974387)
- Gearmotor (already included in the auto chassis): [Gearmotor](https://www.ebay.de/itm/132733015168)
- Raspberry Pi 3b+: [Raspberry Pi 3b+](https://www.amazon.de/dp/B07BDR5PDW/ref=sxnav_sxwds-bovbp-i_m_2?pf_rd_m=A3JWKAKR8XB7XF&pf_rd_p=b3231e2b-a779-4655-bc87-f09acb903eca&pd_rd_wg=0coGd&pf_rd_r=GXQJP37HWMA1E86RFGCP&pf_rd_s=desktop-sx-nav&pf_rd_t=301&pd_rd_i=B07BDR5PDW&pd_rd_w=zJXAB&pf_rd_i=raspberry+pi+3+b%2B&pd_rd_r=063492d1-ecbd-4f0a-b68a-2442561c6d08&ie=UTF8&qid=1535019097&sr=2)
- Arduino UNO: [Arduino UNO](https://www.aliexpress.com/item/1pcs-New-and-original-UNO-R3-ATMega328P-Arduino-UNO-R3-ATMega328-Official-Genuine-with-cable-free/32838136070.html?spm=2114.search0604.3.43.4d833b7fbsJw13&ws_ab_test=searchweb0_0,searchweb201602_1_10320_10152_5724111_10065_10151_10344_10068_10342_10547_5724211_10343_10340_10341_10548_5724311_10696_5724011_10084_10083_10618_10304_10307_10820_10821_10302_10843_10059_100031_10319_10103_5725011_10624_10623_10622_10621_10620_5724911,searchweb201603_2,ppcSwitch_7&algo_expid=efca40dd-c818-4a50-8141-8a9df73b6e4a-9&algo_pvid=efca40dd-c818-4a50-8141-8a9df73b6e4a&priceBeautifyAB=0)
- Picamera: [Picamera](https://de.aliexpress.com/item/NewRaspberry-Pi-Camera-V2-Video-Module-8MP/32664780139.html?spm=a2g0x.search0104.3.78.168f5b50hBP0Oa&ws_ab_test=searchweb0_0,searchweb201602_1_10320_10152_5724111_10065_10151_10344_10068_10342_10547_5724211_10343_10340_10341_10548_5724311_10696_5724011_10084_10083_10618_10304_10307_10820_10821_10302_10843_10059_100031_10319_10103_5725011_10624_10623_10622_10621_10620_5724911,searchweb201603_2,ppcSwitch_7&algo_expid=00222371-cfa6-485d-b494-d56df79776a2-14&algo_pvid=00222371-cfa6-485d-b494-d56df79776a2&priceBeautifyAB=0)
- Ultrasonic ranging module HC-SR04: [Ultrasonic](https://www.aliexpress.com/item/Free-shiping-HC-SR04-HCSR04-to-world-Ultrasonic-Wave-Detector-Ranging-Module-HC-SR04-HC-SR04/32523466302.html?spm=2114.search0604.3.12.18ce45a75xweKn&ws_ab_test=searchweb0_0,searchweb201602_1_10320_10152_5724111_10065_10151_10344_10068_10342_10547_5724211_10343_10340_10341_10548_5724311_10696_5724011_10084_10083_10618_10304_10307_10820_10821_10302_10843_10059_100031_10319_10103_5725011_10624_10623_10622_10621_10620_5724911,searchweb201603_2,ppcSwitch_7&algo_expid=cc35f551-a4f1-45e6-b73f-b071cba8343d-2&algo_pvid=cc35f551-a4f1-45e6-b73f-b071cba8343d&priceBeautifyAB=0)

## Software

## Operating System

## Project Layout
<pre><code>
/hardware
    /chassis
    /sensors

/pictures

/scripts
    os_settings.sh

/software
    /build
    /configs
    /include
    /lib
    /src
    /test
    CMakeLists.txt

LICENSE
README.md

</pre></code>