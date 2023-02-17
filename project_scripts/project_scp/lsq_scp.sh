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
###############################################################################

TAB="    " # 4 Spaces
default_local_path_project="${HOME}/avos3_0/project"
default_path_project="/work/share/project"

default_dir=""
default_dir_bus="RoboBus_CYC"
default_dir_taxi="Robotaxi_CYC"
default_dir_jiangsu_taxi="jiangsulingxing_robotaxi"
default_dir_changan_taxi="PVBU2120_Changan_Robotaxi"
default_dir_chengdu_taxi="A22010P1_Chengdu_Robotaxi"

remote_user="nvidia"
remote_passwd="idriver_bbox"
default_ip="192.168.1.100"
ssh_pass="sshpass -p ${remote_passwd}"

local_path=""
target_path=""

time=$(date "+%Y-%m-%d %H:%M:%S")
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
  info "First Option:"
  info "${TAB}-l|--lib             Transfer devel/lib/* files"
  info "${TAB}-c|--config          Transfer src/config/* files"
  info "${TAB}-m|--map_file        Transfer src/map_file/* files"
  info "${TAB}-s|--script          Transfer script/* files"
  info "${TAB}-csv|--csv           Transfer csv files"
  info "${TAB}-h|--help            Show this message and exit"
  info "Second Option:"
  info "${TAB}-t|--taxi            jiangsulingxing_robotaxi project"
  info "${TAB}-b|--bus             RoboBus_CYC project"
}
###############################################################################

function chose_car() {
  local cmd=$2
  case $cmd in
  -t | --taxi)
    default_dir="${default_dir_taxi}"
    ;;
  -b | --bus)
    default_dir="${default_dir_bus}"
    ;;
  -jst | --jiangsutaxi)
    default_dir="${default_dir_jiangsu_taxi}"
    ;;
  -cat | --changantaxi)
    default_dir="${default_dir_changan_taxi}"
    ;;
  -cdt | --chengdutaxi)
    default_dir="${default_dir_chengdu_taxi}"
    ;;
  *)
    error "Second option invalid input: -t(--taxi) or -b(--bus)"
    exit 0
    ;;
  esac

  local_path="${default_local_path_project}/${default_dir}"
  target_path="${remote_user}@${default_ip}:${default_path_project}/${default_dir}"
  # info "local_path = ${local_path}"
  # info "target_path = ${target_path}"
}

function scp_lib() {
  chose_car $@
  branch_name=$(git rev-parse --abbrev-ref HEAD)  # 获取当前Git仓库中的分支名称
  commit_sha=$(git rev-parse HEAD)  # 获取当前提交的commit值
  # 将分支名称和提交commit值写入名为"branch_info"的文件
  echo "$branch_name" > ./branch_info
  echo "$commit_sha" >> ./branch_info
  ${ssh_pass} scp -r ${local_path}/branch_info ${target_path}/

  start "scp [${GREEN}${default_dir}/devel/lib/*${NO_COLOR}] to [${GREEN}${target_path}/devel/lib/${NO_COLOR}] ······"
  ${ssh_pass} scp -r ${local_path}/devel/lib/* ${target_path}/devel/lib/
  ok "[${GREEN}${default_dir}/devel/lib/*${NO_COLOR}] finish transfer at ${RED}${time}!${NO_COLOR}"

  start "scp [${GREEN}${default_dir}/src/kernel/perception/lib/arm/*${NO_COLOR}] to [${GREEN}${target_path}/src/kernel/perception/lib/arm/${NO_COLOR}] ······"
  ${ssh_pass} scp -r ${local_path}/src/kernel/perception/lib/arm/* ${target_path}/src/kernel/perception/lib/arm/
  ok "[${GREEN}${default_dir}/src/kernel/perception/lib/arm/*${NO_COLOR}] finish transfer at ${RED}${time}!${NO_COLOR}"
}

function scp_config() {
  chose_car $@
  start "scp [${GREEN}${default_dir}/src/config/*${NO_COLOR}] to [${GREEN}${target_path}/src/config/${NO_COLOR}] ······"
  ${ssh_pass} scp -r ${local_path}/src/config/* ${target_path}/src/config/
  ok "[${GREEN}${default_dir}/src/config/*${NO_COLOR}] finish transfer at ${RED}${time}${NO_COLOR}!"
}

function scp_map_file() {
  chose_car $@
  start "scp [${GREEN}${default_dir}/src/map_file/*${NO_COLOR}] to [${GREEN}${target_path}/src/map_file/${NO_COLOR}] ······"
  ${ssh_pass} scp -r ${local_path}/src/map_file/* ${target_path}/src/map_file/
  ok "[${GREEN}${default_dir}/src/map_file/*${NO_COLOR}] finish transfer at ${RED}${time}!${NO_COLOR}"
}

function scp_script() {
  chose_car $@
  start "scp [${GREEN}${default_dir}/script/*${NO_COLOR}] to [${GREEN}${target_path}/src/script/${NO_COLOR}] ······"
  ${ssh_pass} scp -r ${local_path}/script/* ${target_path}/script/
  ok "[${GREEN}${default_dir}/script/*${NO_COLOR}] finish transfer at ${RED}${time}!${NO_COLOR}"
}

function scp_csv() {
  chose_car $@
  start "scp [${GREEN}${target_path}/src/kernel/perception/perception/*.csv${NO_COLOR}] to [${GREEN}${default_dir}/.${NO_COLOR}] ······"
  ${ssh_pass} scp -r ${target_path}/src/kernel/perception/perception/*.csv ./
  ok "[${GREEN}${target_path}/src/kernel/perception/perception/*.csv${NO_COLOR}] finish transfer at ${RED}${time}!${NO_COLOR}"
}

function submoudle_sync() {
  chose_car $@
  cd ${local_path}/script/offline_perception/
  git checkout .
  git checkout develop
  git pull
}
###############################################################################

function main() {
  local cmd=$1
  if [[ "$#" -ne 2 ]]; then
    print_usage
    exit 1
  fi

  case $cmd in
  -h | --help)
    print_usage
    exit 0
    ;;
  -l | --lib)
    scp_lib $@
    exit 0
    ;;
  -c | --config)
    scp_config $@
    exit 0
    ;;
  -m | --map_file)
    scp_map_file $@
    exit 0
    ;;
  -s | --script)
    scp_script $@
    exit 0
    ;;
  -csv | --csv)
    scp_csv $@
    exit 0
    ;;
  -submoudle_sync | --submoudle_sync)
    submoudle_sync $@
    exit 0
    ;;
  *)
    print_usage
    exit 1
    ;;
  esac
}

main $@
###############################################################################
