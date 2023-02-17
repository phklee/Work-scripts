
xhost +local:root 1>/dev/null 2>&1

CONTAINER_NAME=1804_pc
docker exec -u root -it $CONTAINER_NAME /bin/bash

xhost -local:root 1>/dev/null 2>&1


