PRO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
USER_ID=$(id -u)
GRP=$(id -g -n)
GRP_ID=$(id -g)
LOCAL_HOST=$(hostname)
DOCKER_HOME="/home/$USER"

if [ "$USER" == "root" ]; then
  DOCKER_HOME="/root"
fi
if [ ! -d "$HOME/.cache" ]; then
  mkdir "$HOME/.cache"
fi

IMG="192.168.2.95/ktdx/foxy_eloquent_xavier_jetpack4_6:20220225"
CONTAINER_NAME=xavier_2

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
		-v /media:/media \
		-v $HOME/.cache:${DOCKER_HOME}/.cache \
		-v /etc/localtime:/etc/localtime:ro \
  -v $PRO_DIR:/work/share/project \
		--net host \
		--shm-size 512M \
		-w /work/share/project \
		$IMG \
		/bin/bash

