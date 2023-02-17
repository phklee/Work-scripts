
USER_ID=$(id -u)
GRP=$(id -g -n)
GRP_ID=$(id -g)
LOCAL_HOST=`hostname`
DOCKER_HOME="/home/$USER"

if [ "$USER" == "root" ];then
    DOCKER_HOME="/root"
fi
if [ ! -d "$HOME/.cache" ];then
    mkdir "$HOME/.cache"
fi

IMG="192.168.2.95/user-miivii_x86_1804/user-miivii_x86_1804:20200308 "

CONTAINER_NAME=1804_pc

docker run -it \
        -d \
		--privileged \
		--name $CONTAINER_NAME \
		-e DOCKER_USER=$USER \
		-e USER=$USER \
		-e DOCKER_USER_ID=$USER_ID \
		-e DOCKER_GRP=$GRP \
		-e DOCKER_GRP_ID=$GRP_ID \
		--env ROS_DOMAIN_ID=$(date +%N) \
		-v /tmp/.X11-unix:/tmp/.X11-unix:rw \
		-e DISPLAY=unix$DISPLAY \
		-v /media:/media \
		-v $HOME/.cache:${DOCKER_HOME}/.cache \
		-v /etc/localtime:/etc/localtime:ro \
		-v /home/$USER/work:/home/user/work \
		--net host \
		--shm-size 512M \
		-w /home/user/work \
		$IMG \
		/bin/bash
