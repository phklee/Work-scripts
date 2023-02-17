#! /usr/bin/env bash

###############################################################################
#  Copyright (C) 2015-2020, siqi Li
#
#  Description: For rosbag play all module.
#
#  History:
#  lisiqi         2022/09/30    1.0.0    build this module.
###############################################################################

# README
# 参数含义
# -s: segperception模块
# -v: visionperception模块
# -d: dogmperception模块
# 要播放bag的路径: ./robobus_11_20220407172625.bag
# 示例
# ./rosbag_play.sh -s ./robobus_11_20220407172625.bag
# 注意: 必须按照 segperception -> visionperception -> dogmperception 顺序执行

set -e

TAB="    " # 4 Spaces
BOLD='\033[1m'
RED='\033[0;31m'
BLUE='\033[0;34m'
GREEN='\033[32m'
WHITE='\033[34m'
YELLOW='\033[33m'
NO_COLOR='\033[0m'

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
function start() {
  (echo >&2 -e "[${GREEN}${BOLD} STARTING ${NO_COLOR}] $*")
}

# 1. -s: segperception
function segperception() {
  # 用于播放原始包，运行1_visionsegperception_offline.launch
  if [ $# -eq 2 ]; then
    start "${GREEN}rosbag play for segperception"
    # 屏蔽/tpvisionlandet/compressed、/tpvisionsegments/compressed
    rosbag play $2 /tpvisionlandet/compressed:=/LANDET /tpvisionsegments/compressed:=/SEG
  else
    error "${RED}Error param numbers!"
  fi
}

# 2. -v: visionperception
function visionperception() {
  # 用于播放1_segperception.bag，运行2_visionperception_offline.launch
  if [ $# -eq 2 ]; then
    start "${GREEN}rosbag play for visionperception"
    # 屏蔽/tpvisionobjects、/tptrafficlight
    rosbag play $2 /tpvisionobjects:=/OBG /tptrafficlight:=/TRA
  else
    error "${RED}Error param numbers!"
  fi
}

# 3. -d: dogmperception
function dogmperception() {
  # 用于播放2_visionperception.bag，运行3_dogmperception_offline.launch
  if [ $# -eq 2 ]; then
    start "${GREEN}rosbag play for dogmperception"
    # 屏蔽/dogmperception/output
    rosbag play $2 /dogmperception/output:=/DOGM
  else
    error "${RED}Error param numbers!"
  fi
}

# 4. -p1: perception
function perception() {
  # 用于播放3_dogmperception.bag，运行4_perception_offline.launch
  if [ $# -eq 2 ]; then
    start "${GREEN}rosbag play for perception"
    # 关闭毫米波
    # rosbag play $2 /tpperception:=/TP1 /fusion_debug:=/FD /tpars0:=/T0 /tpars1:=/T1 -s 30
    # 打开毫米波
    # rosbag play $2 /tpperception:=/TP1 /fusion_debug:=/FD -s 25 -u 12 -l
    rosbag play $2 /tpperception:=/TP1 /fusion_debug:=/FD -s 2
  else
    error"${RED}Error param numbers!"
  fi
}

# 5. -p2: prediction
function prediction() {
  # 用于播放4_perception.bag，运行5_prediction_offline.launch
  if [ $# -eq 2 ]; then
    start "${GREEN}rosbag play for prediction"
    rosbag play $2 /tpprediction:=/TP2
  else
    error"${RED}Error param numbers!"
  fi
}

# 6. -p3: perception_all
function perception_all() {
  # 用于播放原始.bag，运行0_perception_all.launch
  if [ $# -eq 2 ]; then
    start "${GREEN}rosbag play for perception_all"
    rosbag play $2 /tpvisionlandet/compressed:=/LANDET \
      /tpvisionsegments/compressed:=/SEG \
      /tpvisionobjects:=/OBG \
      /tptrafficlight:=/TRA \
      /dogmperception/output:=/DOGM \
      /tpperception:=/TP1 \
      /tpprediction:=/TP2
  else
    error"${RED}Error param numbers!"
  fi
}

# 7. -p: planning
function planning() {
  # 用于播放原始.bag
  if [ $# -eq 2 ]; then
    start "${GREEN}rosbag play for planning"
    rosbag play $2 --topic /mapengine/tpnavigation \
      /mapengine/tpnavmission \
      /tpimu /tppcican \
      /tpperception \
      /tpprediction \
      /tpcontrolfeedback \
      /miivii_gmsl_ros_node_A/camera/compressed
  else
    error"${RED}Error param numbers!"
  fi
}

function print_usage() {
  info "Usage: $0 [Options]"
  info "Options:"
  info "${TAB}-s |--segperception     rosbag play for segperception module."
  info "${TAB}-v |--visionperception  rosbag play for visionperception module."
  info "${TAB}-d |--dogmperception    rosbag play for dogmperception module."
  info "${TAB}-p1|--perception        rosbag play for play for nch perception module."
  info "${TAB}-p2|--prediction        rosbag play for prediction module."
  info "${TAB}-p3|--prediction_all    rosbag play for all perception module."
  info "${TAB}-p |--planning          rosbag play for all planning module."
  info "${TAB}-h |--help              Show this message and exit."
}

function main() {
  local cmd=$1
  if [[ "$#" -eq 0 ]]; then
    print_usage
    exit 1
  fi

  case $cmd in
  -s | --segperception)
    segperception $@
    exit 0
    ;;
  -v | --visionperception)
    visionperception $@
    exit 0
    ;;
  -d | --dogmperception)
    dogmperception $@
    exit 0
    ;;
  -p1 | --perception)
    perception $@
    exit 0
    ;;
  -p2 | --prediction)
    prediction $@
    exit 0
    ;;
  -p3 | --prediction_all)
    perception_all $@
    exit 0
    ;;
  -p | --planning)
    planning $@
    exit 0
    ;;    
  -h | --help)
    print_usage
    exit 0
    ;;
  *)
    print_usage
    exit 1
    ;;
  esac
}

main $@
