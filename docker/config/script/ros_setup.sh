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
function ROSConfig_fn() {
    SHELL=$(echo ${1} | tr [A-Z] [a-z])
    ROS_TYPE=$(echo ${2} | tr [a-z] [A-Z])
    echo "source /opt/ros/$ROS_DISTRO/setup.${SHELL}" >>/home/$USER/.${SHELL}rc
    echo "export ROS_MASTER_URI=http://${3}:11311" >>/home/$USER/.${SHELL}rc

    if [ ${ROS_TYPE} == "MASTER" ]; then
        echo "export ROS_HOSTNAME=${3}" >>/home/${USER}/.${SHELL}rc
    elif [ ${ROS_TYPE} == "SLAVE" ]; then
        echo "export ROS_HOSTNAME=${4}" >>/home/${USER}/.${SHELL}rc
    fi
}

#################################### MAIN ####################################
ROSConfig_fn "bash" ${ROS_TYPE} ${ROS_MASTER_IP} ${ROS_SLAVE_IP}
ROSConfig_fn "zsh" ${ROS_TYPE} ${ROS_MASTER_IP} ${ROS_SLAVE_IP}
