Date=$(date +"%Y%m%d") # 当前时间: 年月日
IMG="192.168.2.95/user-miivii_cross_1804/user-miivii_cross_1804:${Date}"

CONTAINER_NAME=new_xavier
docker commit $CONTAINER_NAME $IMG


