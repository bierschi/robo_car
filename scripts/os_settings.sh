#!/usr/bin/env bash

check_internet(){
wget -q --spider http://google.com
if [ $? -eq 0 ];
then
   install_ppp_package
else
  echo -e "please check your internet connection and retry!\n"
  exit 1
fi
}

delete_serial_output(){
# delete serial output to console
sed -i s/console=serial0,115200/""/g /boot/cmdline.txt

echo -e "delete serial output to console\n"
}


change_hostname(){
echo
}

#####main#####

echo -e "\nsettings for os starts ...\n"

# first check internet connection
check_internet


echo -e "settings for os finished \n"

echo "reboot starts in 5 seconds"
sleep 5
sudo reboot