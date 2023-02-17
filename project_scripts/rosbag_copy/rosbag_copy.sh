#! /usr/bin/env bash
###############################################################################
#  Copyright (C) 2015-2023, siqi Li
#
#  Description: 用于拷贝车辆上的bag包到本地PC.
#
#  History:
#  lisiqi         2022/12/13    1.0.0    初始版本.
#  lisiqi         2022/12/26    1.0.1    拷贝bag包到指定四个文件夹中..
#  lisiqi         2022/12/28    1.0.2    拷贝bag包到指定一个文件夹中.

# Example:
# 1. ./rosbag_copy.sh -112
# 2. ./rosbag_copy.sh -all
###############################################################################

set -e

BOLD='\033[1m'
RED='\033[0;31m'
BLUE='\033[0;34m'
GREEN='\033[32m'
WHITE='\033[34m'
YELLOW='\033[33m'
NO_COLOR='\033[0m'

###############################################################################

TAB="    " # 4 Spaces
# bag_local_save_path="/home/user/avos3_0/bugs/robobus/boye_k06"
bag_local_save_path=$(pwd)
bag_remote_path="/media/nvidia/user/record_bags/bags/"
date="20230808"
# date=$(date "+%Y%m%d")
txt_file="${date}.txt"

remote_user="nvidia"
remote_passwd="idriver_bbox"
## 本地测试用
# remote_ip_112="192.168.1.100"
# remote_ip_113="192.168.1.100"
# remote_ip_114="192.168.1.100"
# remote_ip_115="192.168.1.100"
remote_ip_112="192.168.1.112"
remote_ip_113="192.168.1.113"
remote_ip_114="192.168.1.114"
remote_ip_115="192.168.1.115"
ssh_pass="sshpass -p ${remote_passwd}"

bbox_name=""
remote_ip=""

###############################################################################

function info() {
  (echo >&2 -e "[${WHITE}${BOLD}INFO${NO_COLOR}] $*")
}
function error() {
  (echo >&2 -e "[${RED}ERROR${NO_COLOR}] $*")
}
function warning() {
  (echo >&2 -e "${YELLOW}[WARNING] $*${NO_COLOR}")
}
function ok() {
  (echo >&2 -e "[${GREEN}${BOLD} OK ${NO_COLOR}] $*")
}
function print_usage() {
  info "Usage: $0 [Options]"
  info "Options:"
  info "${TAB}-112|--112            rosbag_copy_112"
  info "${TAB}-113|--113            rosbag_copy_113"
  info "${TAB}-114|--114            rosbag_copy_114"
  info "${TAB}-115|--115            rosbag_copy_115"
  info "${TAB}-all|--all            rosbag_copy_all"
}

###############################################################################

function bbox_select_112() {
  bbox_name="112ex"
  remote_ip="${remote_ip_112}"
}

function bbox_select_113() {
  bbox_name="113ex"
  remote_ip="${remote_ip_113}"

}

function bbox_select_114() {
  bbox_name="114ex"
  remote_ip="${remote_ip_114}"
}

function bbox_select_115() {
  bbox_name="115ex"
  remote_ip="${remote_ip_115}"
}

function rosbag_copy() {
  remote_path_data="${remote_user}@${remote_ip}:${bag_remote_path}/${date}"
  local_path_data="${bag_local_save_path}/${date}"
  # 若本地文件夹不存在，则新建
  # if [ ! -d "$local_path_data" ]; then
  #   mkdir -p "$local_path_data"
  # fi
  # if [ ! -d "${bag_local_save_path}/${txt_file}" ]; then
  #   mkdir "${bag_local_save_path}/${txt_file}"
  # fi

  # info "remote_path_data = ${remote_path_data}"
  # info "local_path_data = ${local_path_data}"
  # info "bbox_name = ${bbox_name}"
  # info "remote_ip = ${remote_ip}"

  # 循环读取txt文件中的每一行
  while read line; do
    # 判断当前行是否为空白行
    if [[ -z "$line" ]]; then
      # 当前行为空白行
      info "Empty line"
    else
      # 使用下划线作为分隔符，并打印分割后的每一个字段
      IFS='_' read -ra fields <<<"$line"
      bag_name="${fields[0]}_"${bbox_name}"_${fields[2]}_${fields[3]}"
      # info "${bag_name}"
      remote_path="${remote_path_data}/${bag_name}"
      # info "${remote_path}"
      local_path="${local_path_data}/${fields[2]}"
      # info "${local_path}"

      # 若本地文件夹不存在，则新建
      if [ ! -d "$local_path" ]; then
        mkdir -p "$local_path"
      fi

      # 检查bbox ssh 22端口是否可用
      if nc -zvw3 $remote_ip 22; then
        ok "SSH 22 port is open on ${remote_ip}."

        ${ssh_pass} scp -r ${remote_path} ${local_path}
        ok "rosbag copy from ${bbox_name} finish!"
      else
        error "SSH 22 port is closed on $remote_ip."
      fi
    fi
  done <${bag_local_save_path}/${txt_file}
}

function rosbag_copy_112() {
  bbox_select_112
  rosbag_copy
}

function rosbag_copy_113() {
  bbox_select_113
  rosbag_copy
}

function rosbag_copy_114() {
  bbox_select_114
  rosbag_copy
}

function rosbag_copy_115() {
  bbox_select_115
  rosbag_copy
}

function rosbag_copy_all() {
  rosbag_copy_112
  rosbag_copy_113
  rosbag_copy_114
  rosbag_copy_115
}

###############################################################################

function main() {
  local cmd=$1
  if [[ "$#" -eq 0 ]]; then
    print_usage
    exit 1
  fi

  case $cmd in
  -h | --help)
    print_usage
    exit 0
    ;;
  -112 | --112)
    rosbag_copy_112
    exit 0
    ;;
  -113 | --113)
    rosbag_copy_113
    exit 0
    ;;
  -114 | --114)
    rosbag_copy_114
    exit 0
    ;;
  -115 | --115)
    rosbag_copy_115
    exit 0
    ;;
  -all | --all)
    rosbag_copy_all
    exit 0
    ;;
  *)
    print_usage
    exit 1
    ;;
  esac
}

main $@
