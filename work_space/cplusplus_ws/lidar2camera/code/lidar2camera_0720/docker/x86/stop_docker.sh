#docker kill $(docker ps -a -q); docker rm $(docker ps -a -q )
#sudo service docker restart
CONTAINER_NAME=1804_pc

docker stop $CONTAINER_NAME
docker rm $CONTAINER_NAME


