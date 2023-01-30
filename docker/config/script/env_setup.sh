#! /usr/bin/env bash

################################# FUNCTIONS #################################
################################
# Setup ROS master and slave addresses
# Arguments:
# - $1 (String) - Shell Type
# - $2 (String) - This PC is the master or slave
# - $3 (IP) - This IP is the master's IP address
# - $4 (IP) - This IP is the slave's IP address
################################

## add turtlebot model env

function ENVConfig_fn() {
    ## env: turtlebot model (set model as waffle)
    SHELL=$(echo ${1} | tr [A-Z] [a-z])
    echo "export TURTLEBOT3_MODEL=waffle" >>/home/$USER/.${SHELL}rc

    PATH = $(echo "$")
    ## env: set ROS_PACKAGE_PATH
    echo "export ROS_PACKAGE_PATH=\${ROS_PACKAGE_PATH}:/home/$USER/work/src/ORB_SLAM2/Examples/ROS " >>/home/$USER/.${SHELL}rc

    ## env: lidar (permission)
    ##echo "KERNEL=="ttyACM[0-9]*", ACTION=="add", ATTRS{idVendor}=="15d1", MODE="0666", GROUP="dialout", SYMLINK+="sensors/hokuyo"" >> /etc/udev/rules.d/99-hokuyo.rules
    echo "KERNEL=="ttyACM[0-9]*", ACTION=="add", ATTRS{idVendor}=="15d1", MODE="0666", GROUP="dialout", SYMLINK+="sensors/hokuyo"" >> /etc/udev/rules.d/99-hokuyo.rules
    echo "ATTRS{idVendor}=="0483" ATTRS{idProduct}=="5740", ENV{ID_MM_DEVICE_IGNORE}="1", MODE:="0666"" >> /etc/udev/rules.d/99-hokuyo.rules
    echo "ATTRS{idVendor}=="0483" ATTRS{idProduct}=="df11", MODE:="0666"" >> /etc/udev/rules.d/99-hokuyo.rules
    echo "ATTRS{idVendor}=="fff1" ATTRS{idProduct}=="ff48", ENV{ID_MM_DEVICE_IGNORE}="1", MODE:="0666"" >> /etc/udev/rules.d/99-hokuyo.rules
    echo "ATTRS{idVendor}=="10c4" ATTRS{idProduct}=="ea60", ENV{ID_MM_DEVICE_IGNORE}="1", MODE:="0666"" >> /etc/udev/rules.d/99-hokuyo.rules

}

#################################### MAIN ####################################
ENVConfig_fn "bash" ${ROS_TYPE} ${ROS_MASTER_IP} ${ROS_SLAVE_IP}
ENVConfig_fn "zsh" ${ROS_TYPE} ${ROS_MASTER_IP} ${ROS_SLAVE_IP}
