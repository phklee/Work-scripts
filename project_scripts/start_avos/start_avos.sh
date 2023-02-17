#!/bin/bash

# Function: Start_AVOS 
# Date:     2021.06.29
# Version:  V2.1

LOCAL_IP=`hostname -I | cut -d" " -f1`
LOCAL_IP_112="192.168.1.112"
LOCAL_IP_113="192.168.1.113"
LOCAL_IP_114="192.168.1.114"
LOCAL_IP_115="192.168.1.115"
CAR_NO="IDPXJIXX00000D"
PROJECT_NAME="RoboBus_CYC"

function start_AVOS_112()
{
    export ROS_MASTER_URI=http://192.168.1.112:11311
    export ROS_HOSTNAME=192.168.1.112
    rosclean purge -y
    cd /work/share/project/${PROJECT_NAME}/
    source devel/setup.bash
    check_ros_node=`rosnode list | wc -l`
    DATE_NOW=`date`
    echo ${DATE_NOW} "---" $check_ros_node >> /boot/abc.txt
    roslaunch script/${CAR_NO}/start_112.launch > /dev/null 2>&1 
}

function start_AVOS_113()
{
    export ROS_HOSTNAME=192.168.1.113
    export ROS_MASTER_URI=http://192.168.1.112:11311
    rosclean purge -y
    cd /work/share/project/${PROJECT_NAME}/
    source devel/setup.bash

    while true
    do
        check_ros_node=`rosnode list | wc -l`
	DATE_NOW=`date`
	echo ${DATE_NOW} "---" $check_ros_node >> /boot/abc.txt
        if [ $check_ros_node -gt 4 ];
        then
            roslaunch script/${CAR_NO}/start_113.launch > /dev/null 2>&1

            break
        else
            sleep 5
        fi
    done
}

function start_AVOS_114()
{
    export ROS_HOSTNAME=192.168.1.114
    export ROS_MASTER_URI=http://192.168.1.112:11311
    rosclean purge -y
    cd /work/share/project/${PROJECT_NAME}/
    source devel/setup.bash

    while true
    do
        check_ros_node=`rosnode list | wc -l`
	DATE_NOW=`date`
	echo ${DATE_NOW} "---" $check_ros_node >> /boot/abc.txt
        if [ $check_ros_node -gt 4 ];
        then
            roslaunch script/${CAR_NO}/start_114.launch > /dev/null 2>&1

            break
        else
            sleep 5
        fi
    done
}

function start_AVOS_115()
{
    export ROS_HOSTNAME=192.168.1.115
    export ROS_MASTER_URI=http://192.168.1.112:11311
    rosclean purge -y
    cd /work/share/project/${PROJECT_NAME}/
    source devel/setup.bash

    while true
    do
        check_ros_node=`rosnode list | wc -l`
	DATE_NOW=`date`
	echo ${DATE_NOW} "---" $check_ros_node >> /boot/abc.txt
        if [ $check_ros_node -gt 4 ];
        then
            roslaunch script/${CAR_NO}/start_115.launch > /dev/null 2>&1

            break
        else
            sleep 5
        fi
    done
}


if [ "$LOCAL_IP" = "$LOCAL_IP_112" ];
then
	start_AVOS_112
elif [ "$LOCAL_IP" = "$LOCAL_IP_113" ];
then	
	start_AVOS_113
elif [ "$LOCAL_IP" = "$LOCAL_IP_114" ];
then	
	start_AVOS_114
elif [ "$LOCAL_IP" = "$LOCAL_IP_115" ];
then	
	start_AVOS_115
fi

