#!/bin/bash

#
#作者:王磊
#日期:2022.9.2
#功能:从四个xavier中拷贝出bag包
#版本:v1.0
#修改:拷贝路径下有文件时不会自动删除
#

## 
# ./copyBags.sh 20220707174544 2  20220707174544 +- 2落在这个区域之间，会把符合条件的文件拷贝到destination文件夹中.

#指定显示颜色
RED='\E[1;31m'
GREEN='\E[1;32m'
YELLOW='\E[1;33m'
BLUE='\E[1;34m'
PINK='\E[1;35m'
RES='\E[1;0m'

#日志存储位置&需要拷贝的位置
REMOTE_COMPUTER_NAME="nvidia"
XAVIER_LOG_PATH=/media/nvidia/user/record_bags/bags
XAVIER_PASSWORD="idriver_bbox"
remove_yes="-o StrictHostKeyChecking=no"
CURRENT_ADDRESS=$(pwd)
DESTINATION_PATH=${CURRENT_ADDRESS}/destination

#全局变量
g_user_lower_utc=0
g_user_higher_utc=0
g_file_utc=0
g_document_path=/home
g_xavier_number=0

#ip地址
remote_ip_address[0]="192.168.1.112"
remote_ip_address[1]="192.168.1.113"
remote_ip_address[2]="192.168.1.114"
remote_ip_address[3]="192.168.1.115"

IPNumToName() {
  case $1 in
  "192.168.1.112") g_xavier_number=112 ;;
  "192.168.1.113") g_xavier_number=113 ;;
  "192.168.1.114") g_xavier_number=114 ;;
  "192.168.1.115") g_xavier_number=115 ;;
  *) echo -e "${BLUE}输入ip地址错误${RES}" ;;
  esac
  return 0
}

#检查远程源文件目录下是否存在，需要在GetUserUtc()调用后才能调用此函数
function CheckDocumentPath() {
  echo -e "${BLUE}开始检查远程文件是否存在...${res}"
  #$1:远程ip地址
  #检查远程文件路径是否存在
  echo "REMOTE_COMPUTER_NAME= ${REMOTE_COMPUTER_NAME}"
  echo "$1"
  if ! sshpass -p ${XAVIER_PASSWORD} ssh ${remove_yes} ${REMOTE_COMPUTER_NAME}@$1 '[ -e '${g_document_path}' ]'; then
    echo -e "${RED}错误:数据源文件不存在,请检查 (LIN55)${RES}"
    echo -e "${YELLOW}    数据源路径为: ${g_document_path}${RES}"
    echo -e "${YELLOW}    程序退出${RES}"
    exit
  fi

  #TODO:检查远程文件夹内容是否为空
  echo -e "${BLYELLOW}远程文件存在${res}"
}

#拷贝文件
function CopyFiles() {
  #$1:传进来的每一个IP地址
  echo -e "${BLUE}开始拷贝文件，当前拷贝远程IP地址是:$1 请等待...${res}"
  #因为4个控制器的文件名字可能相同，所以需要把不同控制器的数据放到不同文件夹里，具体做法是
  #在destination路径下建立对应控制器的文件夹
  #获取ip地址的最后一位
  IPNumToName $1

  #检查远程文件夹是否存在 defensive
  #CheckDocumentPath $1

  #把ip地址的最后一位组合到拷贝路径中
  destination_path_xavier=${DESTINATION_PATH}/${g_xavier_number}
  echo -e "${PINK}    拷贝的远程路径:${g_document_path}${res}"
  echo -e "${PINK}    拷贝到本地路径为:${destination_path_xavier}${res}"

  #检查文件夹是否存在，不存在就新建一个
  CreatDestinationFiles ${destination_path_xavier}

  for i in $(sshpass -p ${XAVIER_PASSWORD} ssh ${remove_yes} ${REMOTE_COMPUTER_NAME}@$1 ' ls '${g_document_path}' '); do
    GetFileUtc $i
    if [ ${g_file_utc} -le ${g_user_higher_utc} ] && [ ${g_file_utc} -ge ${g_user_lower_utc} ]; then
      echo -e "${PINK}    拷贝文件:$i ${res}"
      sshpass -p ${XAVIER_PASSWORD} scp -r ${REMOTE_COMPUTER_NAME}@$1:${g_document_path}/$i ${destination_path_xavier}
    fi
  done
  echo -e "${YELLOW}拷贝当前文件结束${res}"
}

#获取用户设定的utc时间
function GetUserUtc() {
  echo -e "${BLUE}开始获取用户utc...${RES}"
  local var=$1
  local year=$(echo "${var:0:4}")
  local month=$(echo "${var:4:2}")
  local day=$(echo "${var:6:2}")
  local hour=$(echo "${var:8:2}")
  local minute=$(echo "${var:10:2}")
  local second=$(echo "${var:12:2}")
  local composeDate=$(echo ${year}-${month}-${day})
  local composeTime=$(echo ${hour}:${minute}:${second})
  local utc=$(date +%s -d "${composeDate} ${composeTime}") #把日期转换为utc时间，输入格式必须是，如2022-07-07 10:40:30这样的格式
  local val=$(expr $2 \* 60)
  g_user_lower_utc=$(expr ${utc} - ${val})
  g_user_higher_utc=$(expr ${utc} + ${val})
  g_document_path=${XAVIER_LOG_PATH}/${year}${month}${day}
  #echo "g_document_path= ${g_document_path}"
  #echo "g_user_lower_utc= ${g_user_lower_utc}"
  #echo "g_user_higher_utc= ${g_user_higher_utc}"
  echo -e "${YELLOW}用户utc获取成功${RES}"
}

#获取每个文件的utc时间
function GetFileUtc() {
  local date=$(echo "$1" | awk -F[_] '{print $3}')
  local year=$(echo "${date:0:4}")
  local month=$(echo "${date:4:2}")
  local day=$(echo "${date:6:2}")
  local hour=$(echo "${date:8:2}")
  local minute=$(echo "${date:10:2}")
  local second=$(echo "${date:12:2}")
  local composeDate=$(echo ${year}-${month}-${day})
  local composeTime=$(echo ${hour}:${minute}:${second})
  g_file_utc=$(date +%s -d "${composeDate} ${composeTime}")
}

#判断文件是否为空
function DocumentEmpty() {
  count=$(ls $1 | wc -l)
  if [ $count -gt 0 ]; then
    return 0
  else
    return 1
  fi
}

#判断文件夹是否存在
function DocumentExist() {
  if [ -e $1 ]; then
    return 1
  else
    return 0
  fi
}

#清空目标文件夹下的内容
function CreatDestinationFiles() {
  #$1当前文件路径
  DocumentExist $1
  if [ $? = 0 ]; then
    echo -e "${PINK}    文件夹不存在，新建文件夹$1${RES}"
    mkdir -p $1
  fi
}

#检验输入参数是否全是数字，用作安全检查
function ValidNum() {
  validNUm="$(echo $1 | sed -e 's/[^[:digit:]]//g')" #如果输入有非数字，替换掉
  if [ "$validNUm" = $1 ]; then
    return 1
  else
    return 0
  fi
}

function ErrorCheck() {
  echo -e "${BLUE}开始检查用户输入...${RES}"
  #错误检查
  if [ $# -lt 2 ]; then
    echo -e "${RED}错误:${RES}"
    echo -e "${YELLOW}    调用格式为:$0 date interval${RES}"
    echo -e "${YELLOW}    例如:./copyBags.sh 20220707174544 2${RES}"
    echo -e "${YELLOW}    程序退出${RES}"
    exit
  fi

  ValidNum $1
  if [ $? = 0 ]; then
    echo -e "${RED}错误:输入日期格式错误${RES}"
    echo -e "${YELLOW}    当前输入为: $1"
    echo -e "${YELLOW}    应该是这样的格式: 20220707174544"
    echo -e "${YELLOW}    程序退出${RES}"
    exit
  fi

  ValidNum $2
  if [ $? = 0 ]; then
    echo -e "${RED}错误:间隔时间输入错误，应该是整数${RES}"
    echo -e "${YELLOW}    当前间隔: $2"
    echo -e "${YELLOW}    程序退出${RES}"
    exit
  fi

  local var=$1
  local len=${#var}
  if [ $len -ne 14 ]; then
    echo -e "${RED}错误:输入日期长度不正确${RES}"
    echo -e "${YELLOW}    当前输入: $1"
    echo -e "${YELLOW}    应该是这样的格式: 20220707174544"
    echo -e "${YELLOW}    程序退出${RES}"
    exit
  fi
  echo -e "${YELLOW}用户输入检查成功${RES}"
}

function CheckNetWork() {
  echo -e "${BLUE}开始检查网络...${RES}"
  #目前仅检查112
  ping=$(ping -c 1 ${remote_ip_address[0]} | grep loss | awk '{print $6}')
  success=$(echo ${ping} | awk -F[%] '{print $1}')

  if [ $success -eq 100 ]; then
    echo -e "${RED}错误:程序与主机不能ping通，请检查网络${RES}"
    echo -e "${YELLOW}    当前主机IP: ${remote_ip_address[0]}"
    echo -e "${YELLOW}    程序退出${RES}"
    exit
  fi
  echo -e "${YELLOW}网络检查成功${RES}"
}

function Init() {
  if [ $1 = "-h" ]; then
    echo -e "${BLUE}********************************************************${RES}"
    echo -e "${PINK}@功能说明:根据输入的时间及间隔信息拷贝对应时间段的日志文件 ${RES}"
    echo -e "${PINK}@硬件连接:从笔记本电脑连接网线到控制器,并把笔记本电脑的    ${RES}"
    echo -e "${PINK}          网段设置为192.168.1.***,注意笔记本设置的网段    ${RES}"
    echo -e "${PINK}          不要和控制器冲突                               ${RES}"
    echo -e "${PINK}@依赖: 如果提示没有sshpass，需要先安装                    ${RES}"
    echo -e "${PINK}       sudo apt-get install sshpass                      ${RES}"
    echo -e "${PINK}@指令格式: ./copyBags.sh date interval${RES}"
    echo -e "${PINK}@参数: date:输入年月日时分秒的日期格式${RES}"
    echo -e "${PINK}@参数: interval:输入整数间隔时间,单位(分)${RES}"
    echo -e "${PINK}@示例: ./copyBags.sh 20220707174544 2${RES}"
    echo -e "${PINK}@示例说明:拷贝20220707174344-20220707174744四分钟内的数据${RES}"
    echo -e "${BLUE}********************************************************${RES}"
    exit
  fi
  CheckNetWork
  ErrorCheck $@
  GetUserUtc $@
}

function main() {
  Init $@

  for i in ${remote_ip_address[*]}; do
    CopyFiles $i
  done

  echo -e "${BLUE}Done${res}"
}

main $@
