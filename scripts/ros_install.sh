#!/usr/bin/env bash

check_internet(){
wget -q --spider http://google.com
if [ $? -eq 0 ];
then
  echo -e "internet connection available!\n"
else
  echo -e "please check your internet connection and retry!\n"
  exit 1
fi
}

setup_ros_repositories(){

echo -e "\nsetup ros repositories!\n"

sudo apt-get install dirmngr
sudo apt-get update
sudo apt-get upgrade

}

install_bootstrap_dependencies(){

echo -e "\ninstall bootstrap dependencies!\n"

sudo apt-get install -y python-rosdep python-rosinstall-generator python-wstool python-rosinstall build-essential cmake

}

initializing_rosdep(){

echo -e "\ninitializing rosdep!\n"

sudo rosdep init
rosdep update

}

create_catkin_workspace(){

echo -e "\ncreate catkin workspace!\n"

mkdir -p ~/ros_catkin_ws
cd ~/ros_catkin_ws

}

core_packages(){

echo -e "\ninstall core packages!\n"

rosinstall_generator ros_comm --rosdistro kinetic --deps --wet-only --tar > kinetic-ros_comm-wet.rosinstall
wstool init src kinetic-ros_comm-wet.rosinstall

}

core_and_desktop_packages(){

echo -e "\ninstall core and desktop packages!\n"

rosinstall_generator desktop --rosdistro kinetic --deps --wet-only --tar > kinetic-desktop-wet.rosinstall
wstool init src kinetic-desktop-wet.rosinstall
}


resolve_dependencies(){

echo -e "\nresolve dependencies!\n"

mkdir -p ~/ros_catkin_ws/external_src
cd ~/ros_catkin_ws/external_src
wget http://sourceforge.net/projects/assimp/files/assimp-3.1/assimp-3.1.1_no_test_models.zip/download -O assimp-3.1.1_no_test_models.zip
unzip assimp-3.1.1_no_test_models.zip
cd assimp-3.1.1
cmake .
make
sudo make install

}

resolve_dependencies_with_rosdep(){

echo -e "\nresolve dependencies with rosdep!\n"

cd ~/ros_catkin_ws
rosdep install -y --from-paths src --ignore-src --rosdistro kinetic -r --os=debian:stretch

}

building_catkin_ws(){

echo -e "\nbuilding catkin workspace!\n"

sudo pip install --upgrade catkin_pkg_modules
sudo ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/kinetic -j2

}

setup_bashrc(){

echo -e "\nsetup bashrc!\n"

echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc

}

ros_install_version(){
read -rep $'Which version of ros shall be installed? Type: 1 (only ros core packages), Type: 2 (ros core and desktop packages)\n' numbers

if (( $numbers > 0 )) && (( $numbers < 3 ))
then
    # prerequisites
    setup_ros_repositories
    install_bootstrap_dependencies
    initializing_rosdep

    # installation
    create_catkin_workspace

  if [ $numbers -eq 1 ]
  then
    core_packages
    resolve_dependencies
    resolve_dependencies_with_rosdep
    building_catkin_ws
    setup_bashrc

  elif [ $numbers -eq 2 ]
  then
    core_and_desktop_packages
    resolve_dependencies
    resolve_dependencies_with_rosdep
    building_catkin_ws
    setup_bashrc
  fi
else
  echo -e "wrong number, please retry!\n"
  ros_install_version
fi
}

##### main #####

echo -e "\ninstall ros ...\n"
# first check internet connection
check_internet
ros_install_version


echo -e "successfully installed ros kinetic \n"

echo "reboot starts in 5 seconds"
sleep 5
sudo reboot