#! /usr/bin/env bash

###############################################################################
#  Copyright (C) 2015-2020, siqi Li
#
#  Description: For rosbag record all module messages.
#
#  History:
#  lisiqi         2022/09/30    1.0.0    build this module.
###############################################################################

# README
# 将各模块的显示话题都记录
# 参数含义
# -s: segperception模块
# -v: visionperception模块
# -d: dogmperception模块
# -p1: perception模块
# -p2: prediction模块
# 示例
# ./record_bag_for_KTD_visual-all.sh -s
# 注意: 必须按照 segperception -> visionperception -> dogmperception -> perception -> prediction 顺序执行

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
  if [ $# -eq 2 ]; then
    start "${GREEN}rosbag record for segperception"
    # 重点记录/tpvisionlandet/compressed、/tpvisionsegments/compressed
    # 用于显示debug，记录/debug/vision_resa_result(车道线分割)、/debug/vision_segment_result(视觉分割)
    rosbag record /driver/lidar/rs_80_packets/top_center \
      /driver/lidar/rs_helios_packets/top_center \
      /driver/lidar/vlp_32_packets/center \
      /driver/lidar/rs_bp_packets/left \
      /driver/lidar/rs_bp_packets/right \
      /driver/lidar/rs_helios_packets/left \
      /driver/lidar/rs_helios_packets/right \
      /driver/lidar/vlp_16_packets/left \
      /driver/lidar/vlp_16_packets/right \
      /driver/lidar/rs_lidar_packets/left \
      /driver/lidar/rs_lidar_packets/right \
      /driver/lidar/tw_0332_packets/front \
      /driver/lidar/tw_0332_packets/rear \
      /driver/lidar/tw_scope192a2_packets/front \
      /driver/lidar/tw_scope192a2_packets/rear \
      /driver/lidar/horizon_packets/rear_left \
      /driver/lidar/horizon_packets/rear_right \
      /filter_status_localization \
      /ivsensorgps \
      /ivsensorimu \
      /lidarmatching \
      /mapengine/tpnavigation \
      /mapengine/tpnavmission \
      /miivii_gmsl_ros_node_A/camera/compressed \
      /target_reference_line \
      /tpars0 \
      /tpars1 \
      /tpbusiness_cmd \
      /tpbusiness_status \
      /tpcanfeedback \
      /tpcontrol \
      /tpcontrolfeedback \
      /tpimu \
      /tppathplan \
      /tppcican \
      /tptask_fb \
      /tpvisionlandet/compressed \
      /tpvisionsegments/compressed \
      /debug/vision_resa_result \
      /debug/vision_segment_result -o ${2}/1_segperception.bag
  else
    error "${RED}Error param numbers!"
  fi
}

# 2. -v: visionperception
function visionperception() {
  if [ $# -eq 2 ]; then
    start "${GREEN}rosbag record for visionperception"
    # 重点记录/tptrafficlight、/tpvisionobjects
    # 用于显示debug，记录/debug/object_detect_result(目标检测识别)、/debug/trafficlight_results(红绿灯检测识别)
    rosbag record /driver/lidar/rs_80_packets/top_center \
      /driver/lidar/rs_helios_packets/top_center \
      /driver/lidar/vlp_32_packets/center \
      /driver/lidar/rs_bp_packets/left \
      /driver/lidar/rs_bp_packets/right \
      /driver/lidar/rs_helios_packets/left \
      /driver/lidar/rs_helios_packets/right \
      /driver/lidar/vlp_16_packets/left \
      /driver/lidar/vlp_16_packets/right \
      /driver/lidar/rs_lidar_packets/left \
      /driver/lidar/rs_lidar_packets/right \
      /driver/lidar/tw_0332_packets/front \
      /driver/lidar/tw_0332_packets/rear \
      /driver/lidar/tw_scope192a2_packets/front \
      /driver/lidar/tw_scope192a2_packets/rear \
      /driver/lidar/horizon_packets/rear_left \
      /driver/lidar/horizon_packets/rear_right \
      /filter_status_localization \
      /ivsensorgps \
      /ivsensorimu \
      /lidarmatching \
      /mapengine/tpnavigation \
      /mapengine/tpnavmission \
      /miivii_gmsl_ros_node_A/camera/compressed \
      /target_reference_line \
      /tpars0 \
      /tpars1 \
      /tpbusiness_cmd \
      /tpbusiness_status \
      /tpcanfeedback \
      /tpcontrol \
      /tpcontrolfeedback \
      /tpimu \
      /tppathplan \
      /tppcican \
      /tptask_fb \
      /tptrafficlight \
      /tpvisionlandet/compressed \
      /tpvisionobjects \
      /tpvisionsegments/compressed \
      /debug/vision_resa_result \
      /debug/vision_segment_result \
      /debug/object_detect_result \
      /debug/trafficlight_results -o ${2}/2_visionperception.bag
  else
    error "${RED}Error param numbers!"
  fi
}

# 3. -d: dogmperception
function dogmperception() {
  if [ $# -eq 2 ]; then
    start "${GREEN}rosbag record for dogmperception"
    # 重点记录/dogmperception/output
    # 用于显示debug，记录/dogm/debug/no_ground_cloud
    rosbag record /driver/lidar/rs_80_packets/top_center \
      /driver/lidar/rs_helios_packets/top_center \
      /driver/lidar/vlp_32_packets/center \
      /driver/lidar/rs_bp_packets/left \
      /driver/lidar/rs_bp_packets/right \
      /driver/lidar/rs_helios_packets/left \
      /driver/lidar/rs_helios_packets/right \
      /driver/lidar/vlp_16_packets/left \
      /driver/lidar/vlp_16_packets/right \
      /driver/lidar/rs_lidar_packets/left \
      /driver/lidar/rs_lidar_packets/right \
      /driver/lidar/tw_0332_packets/front \
      /driver/lidar/tw_0332_packets/rear \
      /driver/lidar/tw_scope192a2_packets/front \
      /driver/lidar/tw_scope192a2_packets/rear \
      /driver/lidar/horizon_packets/rear_left \
      /driver/lidar/horizon_packets/rear_right \
      /filter_status_localization \
      /ivsensorgps \
      /ivsensorimu \
      /lidarmatching \
      /mapengine/tpnavigation \
      /mapengine/tpnavmission \
      /miivii_gmsl_ros_node_A/camera/compressed \
      /target_reference_line \
      /tpars0 \
      /tpars1 \
      /tpbusiness_cmd \
      /tpbusiness_status \
      /tpcanfeedback \
      /tpcontrol \
      /tpcontrolfeedback \
      /tpimu \
      /tppathplan \
      /tppcican \
      /tptask_fb \
      /tptrafficlight \
      /tpvisionlandet/compressed \
      /tpvisionobjects \
      /tpvisionsegments/compressed \
      /dogmperception/output \
      /debug/vision_resa_result \
      /debug/vision_segment_result \
      /debug/object_detect_result \
      /debug/trafficlight_results \
      /dogm/debug/no_ground_cloud \
      /dogm_cluster/dogm_clouds \
      /dogm_cluster/dogmdet -o ${2}/3_dogmperception.bag
  else
    error "${RED}Error param numbers!"
  fi
}

# 4. -p1: perception
function perception() {
  if [ $# -eq 2 ]; then
    start "${GREEN}rosbag record for perception"
    # 重点记录/tpperception、/lidar/debug/cluster_bbox(激光检测)
    rosbag record /driver/lidar/rs_80_packets/top_center \
      /driver/lidar/rs_helios_packets/top_center \
      /driver/lidar/vlp_32_packets/center \
      /driver/lidar/rs_bp_packets/left \
      /driver/lidar/rs_bp_packets/right \
      /driver/lidar/rs_helios_packets/left \
      /driver/lidar/rs_helios_packets/right \
      /driver/lidar/vlp_16_packets/left \
      /driver/lidar/vlp_16_packets/right \
      /driver/lidar/rs_lidar_packets/left \
      /driver/lidar/rs_lidar_packets/right \
      /driver/lidar/tw_0332_packets/front \
      /driver/lidar/tw_0332_packets/rear \
      /driver/lidar/tw_scope192a2_packets/front \
      /driver/lidar/tw_scope192a2_packets/rear \
      /driver/lidar/horizon_packets/rear_left \
      /driver/lidar/horizon_packets/rear_right \
      /filter_status_localization \
      /fusion_debug \
      /ivsensorgps \
      /ivsensorimu \
      /lidarmatching \
      /mapengine/tpnavigation \
      /mapengine/tpnavmission \
      /miivii_gmsl_ros_node_A/camera/compressed \
      /target_reference_line \
      /tpars0 \
      /tpars1 \
      /radarprocess/raw_object \
      /radarprocess/raw_object/text \
      /radarprocess/object \
      /radarprocess/object/text \
      /radarprocess/cluster \
      /radarprocess/cluster/text \
      /radarprocess/track \
      /radarprocess/track/text \
      /radarprocess/fusion \
      /radar2cam/image \
      /tpbusiness_cmd \
      /tpbusiness_status \
      /tpcanfeedback \
      /tpcontrol \
      /tpcontrolfeedback \
      /tpimu \
      /tppathplan \
      /tppcican \
      /tpperception \
      /tptask_fb \
      /tptrafficlight \
      /tpvisionlandet/compressed \
      /tpvisionobjects \
      /tpvisionsegments/compressed \
      /dogmperception/output \
      /debug/vision_resa_result \
      /debug/vision_segment_result \
      /debug/object_detect_result \
      /debug/trafficlight_results \
      /lidar/debug/cluster_bbox \
      /lidar/debug/cluster_bbox_text \
      /lidar/debug/cluster_polygon \
      /dogm/debug/no_ground_cloud \
      /dogm_cluster/dogmdet \
      /dogm_cluster/dogm_clouds \
      /lidar/debug/no_ground_cloud \
      /fusion/semantic_image \
      /lidar2object/cam_det \
      /lidar2object/cam_trk \
      /lidar2object/detections_lidar \
      /lidar2object/cloudsImg \
      /fusion/lidardet \
      /fusion/radardet \
      /fusion/dogmdet \
      /fusion/object_colored_clouds -o ${2}/4_perception.bag

  else
    error "${RED}Error param numbers!"
  fi
}

# 5. -pc: perception_for_control,感知仿真后,记录控制模块仿真所需话题
function perception_for_control() {
  if [ $# -eq 2 ]; then
    start "${GREEN}rosbag record for perception_for_control"
    # 重点记录/tpperception
    rosbag record /tpimu \
      /tpperception \
      /tppathplan \
      /tpcanfeedback -o ${2}/4_perception_for_control.bag
  else
    error "${RED}Error param numbers!"
  fi
}

# 6. -p2: prediction
function prediction() {
  if [ $# -eq 2 ]; then
    start "${GREEN}rosbag record for prediction"
    # 重点记录/tpprediction
    rosbag record /driver/lidar/rs_80_packets/top_center \
      /driver/lidar/rs_helios_packets/top_center \
      /driver/lidar/vlp_32_packets/center \
      /driver/lidar/rs_bp_packets/left \
      /driver/lidar/rs_bp_packets/right \
      /driver/lidar/rs_helios_packets/left \
      /driver/lidar/rs_helios_packets/right \
      /driver/lidar/vlp_16_packets/left \
      /driver/lidar/vlp_16_packets/right \
      /driver/lidar/rs_lidar_packets/left \
      /driver/lidar/rs_lidar_packets/right \
      /driver/lidar/tw_0332_packets/front \
      /driver/lidar/tw_0332_packets/rear \
      /driver/lidar/tw_scope192a2_packets/front \
      /driver/lidar/tw_scope192a2_packets/rear \
      /driver/lidar/horizon_packets/rear_left \
      /driver/lidar/horizon_packets/rear_right \
      /filter_status_localization \
      /fusion_debug \
      /ivsensorgps \
      /ivsensorimu \
      /lidarmatching \
      /mapengine/tpnavigation \
      /mapengine/tpnavmission \
      /miivii_gmsl_ros_node_A/camera/compressed \
      /target_reference_line \
      /tpars0 \
      /tpars1 \
      /tpbusiness_cmd \
      /tpbusiness_status \
      /tpcanfeedback \
      /tpcontrol \
      /tpcontrolfeedback \
      /tpimu \
      /tppathplan \
      /tppcican \
      /tpperception \
      /tptask_fb \
      /tptrafficlight \
      /tpvisionlandet/compressed \
      /tpvisionobjects \
      /tpvisionsegments/compressed \
      /dogmperception/output \
      /debug/vision_resa_result \
      /debug/vision_segment_result \
      /debug/object_detect_result \
      /debug/trafficlight_results \
      /dogm/debug/no_ground_cloud \
      /lidar/debug/cluster_bbox \
      /fusion/semantic_image \
      /fusion/lidardet \
      /fusion/radardet \
      /fusion/dogmdet \
      /fusion/cloudsImg \
      /fusion/object_colored_clouds \
      /planningdebug \
      /tpprediction -o ${2}/5_prediction.bag
  else
    error "${RED}Error param numbers!"
  fi
}

# 7. -pp: prediction_for_planning, 感知->预测仿真后,记录规划和认知决策仿真所需话题
function prediction_for_planning() {
  if [ $# -eq 2 ]; then
    start "${GREEN}rosbag record for prediction_for_planning"
    # 重点记录/tpprediction
    rosbag record /tpimu \
      /tpperception \
      /tpprediction \
      /tpcontrolfeedback \
      /mapengine/tpnavigation \
      /mapengine/tpnavmission \
      /miivii_gmsl_ros_node_A/camera/compressed \
      /tppcican -o ${2}/5_prediction_for_planning.bag
  else
    error "${RED}Error param numbers!"
  fi
}

# 8. -p: planning, 规划仿真,记录相关可视化话题
function planning() {
  if [ $# -eq 2 ]; then
    start "${GREEN}rosbag record for planning"
    # 重点记录/tppathplan
    rosbag record /tpimu \
      /tpconbox \
      /mapengine/tpnavigation \
      /mapengine/tpnavmission \
      /tppcican \
      /tpcontrol \
      /lon_control_debug \
      /lat_control_debug \
      /lat_control_debug_pt \
      /lon_control_debug_pt \
      /control_sys \
      /stmap_debug \
      /planningdebug \
      /planning_debug/xt_boundary_debug \
      /tppathplan \
      /target_reference_line \
      /tpperception \
      /tpprediction \
      /prediction_debug/lanenet \
      /prediction_debug/junctionnet \
      /prediction_debug/objectsfeature \
      /miivii_gmsl_ros_node_A/camera/compressed \
      /tplanelines -o ${2}/6_planning.bag 
  else
    error "${RED}Error param numbers!"
  fi
}

function print_usage() {
  info "Usage: $0 [Options]"
  info "Options:"
  info "${TAB}-s |--segperception             rosbag record for segperception module."
  info "${TAB}-v |--visionperception          rosbag record for visionperception module."
  info "${TAB}-d |--dogmperception            rosbag record for dogmperception module."
  info "${TAB}-p1|--perception                rosbag record for perception module."
  info "${TAB}-pc|--perception_for_control    rosbag record for perception_for_control module."
  info "${TAB}-p2|--prediction                rosbag record for prediction module."
  info "${TAB}-pp|--prediction_for_planning   rosbag record for prediction_for_planning module."
  info "${TAB}-p |--planning                  rosbag record for planning module."
  info "${TAB}-h |--help                      Show this message and exit."
}

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
  -pc | --perception_for_control)
    perception_for_control $@
    exit 0
    ;;
  -p2 | --prediction)
    prediction $@
    exit 0
    ;;
  -pp | --prediction_for_planning)
    prediction_for_planning $@
    exit 0
    ;;
  -p | --planning)
    planning $@
    exit 0
    ;;    
  *)
    print_usage
    exit 1
    ;;
  esac
}

main $@
