GREEN='\e[0;32m'
RED='\033[0;31m'
BLUE='\033[0;34m'
BOLD='\033[1m'
NONE='\033[0m'
check_status=0
echo "ready to check  " $1 $2
target_name=$1
# target_name="user";
target_ip=$2 #`echo $ROS_MASTER_URI |awk -F ':11311' '{print $1}'| awk '{split($0, array, "//");print array[2]}'`;
if [[ "$target_ip" == "localhost" ]]; then
  target_ip="127.0.0.1"
fi
remove_yes="-o StrictHostKeyChecking=no"

function SetupEnv() {
  target_file="/home/$USER/.ssh/id_rsa.pub"
  if [[ "$USER" == "root" ]]; then
    target_file="/root/.ssh/id_rsa.pub"
  fi
  echo "cheching target_file  $target_file"
  if [ ! -r "$target_file" ]; then
    echo -e "${RED} ~/.ssh/id_rsa.pub please run ssh-keygen to generate the ssh key ${NONE}"
    return 1
  fi
  pub_key=$(cat $target_file)
  echo $pub_key
  cmd="echo '$pub_key' >> ~/.ssh/authorized_keys"
  echo "ready to exe ${cmd}"
  echo -e "$BOLD need to enter password : ${NONE}"
  ssh ${remove_yes} ${target_name}@${target_ip} -t ${cmd}
  if [ $? != 0 ]; then
    echo -e "${RED}SetupEnv ${target_name}@${target_ip} ${NONE}"
    return 1
  else
    echo -e "${GREEN}setup env success!${NONE}"
    return 0
  fi
}

function CheckTargetIP() {
  starttime_ip=$(date +'%Y-%m-%d %H:%M:%S')
  start_seconds_ip=$(date --date="$starttime_ip" +%s)
  echo "timeout 10 ping -c 1 ${target_ip}"
  timeout 10 ping -c 1 ${target_ip}
  endtime_ip=$(date +'%Y-%m-%d %H:%M:%S')
  end_seconds_ip=$(date --date="$endtime_ip" +%s)
  if [[ $((end_seconds_ip - start_seconds_ip)) -gt 7 ]]; then

    return 1
  fi
  return 0
}

function CheckEnv() {
  starttime=$(date +'%Y-%m-%d %H:%M:%S')
  start_seconds=$(date --date="$starttime" +%s)
  echo -e "$BOLD do not enter password ,just wait ${NONE}"
  timeout 5 ssh ${target_name}@${target_ip} -t "date"
  echo "timeout 5 ssh ${target_name}@${target_ip} -t \"date\""
  endtime=$(date +'%Y-%m-%d %H:%M:%S')
  end_seconds=$(date --date="$endtime" +%s)
  echo $((end_seconds - start_seconds))
  if [[ $((end_seconds - start_seconds)) -gt 3 ]]; then
    echo -e "$BLUE environment is not meet demand, will exe ssh authorized_keys$NONE"
    return 1
  fi
  return 0
}

#timeout 10 ssh ${target_name}@${target_ip} -t "date"

echo $target_ip

if [[ "$target_ip" != "127.0.0.1" ]]; then
  CheckTargetIP
  if [[ $? != 0 ]]; then
    echo -e "${RED}target_ip is not reachable,please check $target_ip ${NONE}"
  else
    CheckEnv
    if [[ $? != 0 ]]; then
      SetupEnv
      if [[ $? != 0 ]]; then
        echo -e "${RED}SetupEnv error${NONE}"
        return

      fi
      CheckEnv
      if [[ $? != 0 ]]; then
        echo -e "${RED}SetupEnv error please contact xiayang@user.com${NONE}"
        return
      else
        echo -e "${GREEN}your environment is ok now${NONE}"
      fi
    else
      echo -e "${GREEN}your environment is ok${NONE}"
    fi
  fi

fi

ssh ${remove_yes} ${target_name}@${target_ip} -t "sudo chmod -R 777 /work"
