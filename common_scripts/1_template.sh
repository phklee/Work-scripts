#!/bin/bash
###############################################################################
#  Copyright (C) 2015-2023, siqi Li
#
#  Description: 快速编译生成modules_export感知所需文件夹.
#  Use: ./build.sh make
#
#  History:
#  lisiqi         2023/02/13    1.0.0    初始版本.
###############################################################################
set -e
# Color Settings
BOLD='\033[1m'
RED='\033[0;31m'
BLUE='\033[0;34m'
GREEN='\033[32m'
WHITE='\033[34m'
YELLOW='\033[33m'
NO_COLOR='\033[0m'
TAB="    " # 4 Spaces
###############################################################################
# Info Print Settings
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
  info "${TAB}make           Build all needed modules"  
  info "${TAB}clear_all      Delete all build files"
}
###############################################################################
# Directory Settings
project_dir="/work/share/project/perception_lidar_det"
export_name="modules_export"
###############################################################################
# Function Settings
function build_all(){
  if [! -d "${project_dir}/build"]
  then
    mkdir "${project_dir}/build"
  fi

  cd ${project_dir}/build
  ## 编译后的库和头文件会安装到modules_export目录
  cmake ../modules -DCMAKE_INSTALL_PREFIX=../${export_name}
  make
  make install
  cd ${project_dir}
  ok "Build all needed modules successful!"
}

function clear_all(){
  rm -rf ${project_dir}/build
  rm -rf ${project_dir}/${export_name}
  ok "Delete all build files successful!"
}
###############################################################################
# Main Function
function main() {
  local cmd=$1
  if [[ "$#" -ne 1 ]]
  then
    print_usage
    exit 1
  fi

  case $cmd in
    -h | --help)
      print_usage
      exit 0
      ;;
    make)
      build_all $@
      exit 0
      ;;
    clear_all)
      clear_all $@
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