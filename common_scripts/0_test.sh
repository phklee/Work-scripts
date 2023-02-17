#! /usr/bin/env bash
###############################################################################
#  Copyright (C) 2015-2023, siqi Li
#
#  Description: 用于BBOX和本地PC之间拷贝文件.
#
#  History:
#  lisiqi         2022/09/30    1.0.0    初始版本.
#  lisiqi         2022/12/15    1.0.1    格式化脚本.
#  lisiqi         2023/01/10    1.0.2    增加拷贝csv文件函数.
#  lisiqi         2023/01/11    1.0.3    适配Taxi和Bus.
###############################################################################
set -e

BOLD='\033[1m'
RED='\033[0;31m'
BLUE='\033[0;34m'
GREEN='\033[32m'
WHITE='\033[34m'
YELLOW='\033[33m'
NO_COLOR='\033[0m'
TAB="    " # 4 Spaces
###############################################################################

default_local_path_project="${HOME}/avos3_0/project"
default_path_project="/work/share/project"

default_dir=""
default_dir_bus="RoboBus_CYC"
default_dir_taxi="jiangsulingxing_robotaxi"

remote_user="nvidia"
remote_passwd="idriver_bbox"
default_ip="192.168.1.100"
ssh_pass="sshpass -p ${remote_passwd}"

local_path=""
target_path=""
###############################################################################

function info() {
  (echo >&2 -e "[${WHITE}${BOLD}INFO${NO_COLOR}] $*")
}
function start() {
  (echo >&2 -e "[${WHITE}${BOLD}START${NO_COLOR}] $*")
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
}
###############################################################################


###############################################################################

function main() {

	info "\$#: number = $#"
	info "\$0: scname = $0"
	info "\$1: first  = $1"
	info "\$2: second = $2"
	info "\$@: argume = $@"

  if [[ "$#" -ne 2 ]]; then 
    info "$@"
  fi

}

main $@
###############################################################################