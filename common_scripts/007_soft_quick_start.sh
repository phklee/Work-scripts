#!/usr/bin/env bash
###############################################################################
#  Copyright (C) 2015-2023, lisiqi
#
#  Description: 用于快速启动本地软件.
#
#  History:
#  lisiqi         2023/02/06    1.0.0    初始版本.
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
  info "Usage: $0"
  info "${TAB}start_wechat        启动微信"
  info "${TAB}stop_wechat         关闭微信"
  info "${TAB}start_clash         启动Clash-VPN"
  info "${TAB}start_iodeclient    启动iNodeClient-VPN"
  info "${TAB}start_memos         启动memos"
  info "${TAB}stop_memos          关闭memos"
}
###############################################################################

function start_wechat() {
  /home/user/Software/docker/wechat/start_wechat.sh
}

function stop_wechat() {
  /home/user/Software/docker/wechat/stop_wechat.sh
}

function start_clash() {
  /home/user/Software/clash/cfw
}

function start_inodeclient() {
  /home/user/work/idriver_/2_tools/2_soft/iNodeClient/iNodeClient.sh
}

function start_memos() {
  /home/user/work/codes_/github_/memos/docker/start_docker.sh
}

function stop_memos() {
  /home/user/work/codes_/github_/memos/docker/stop_docker.sh
}
###############################################################################

function main() {
  local cmd=$1
  if [[ "$#" -ne 1 ]]; then
    print_usage
    exit 1
  fi

  case $cmd in
  -h | --help)
    print_usage
    exit 0
    ;;
  --start_wechat)
    start_wechat
    exit 0
    ;;
  --stop_wechat)
    stop_wechat
    exit 0
    ;;
  --start_clash)
    start_clash
    exit 0
    ;;
  --start_inodeclient)
    start_inodeclient
    exit 0
    ;;
  --start_memos)
    start_memos
    exit 0
    ;;
  --stop_memos)
    stop_memos
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
