#! /usr/bin/env bash
###############################################################################
#  Copyright (C) 2015-2020, siqi Li
#
#  Description: For fast transfer of module files.
#
#  History:
#  lisiqi         2022/09/30    1.0.0    build this module.
###############################################################################
set -e

# TOP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
# echo ${TOP_DIR}
# source "/home/user/work/project_/github_/apollo/scripts/apollo.bashrc"

TAB="    " # 4 Spaces
BOLD='\033[1m'
RED='\033[0;31m'
BLUE='\033[0;34m'
GREEN='\033[32m'
WHITE='\033[34m'
YELLOW='\033[33m'
NO_COLOR='\033[0m'
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
###############################################################################

function main() {
  # 获取要格式化的文件名
  filename=$1

  # 获取 .clang-format 文件的名称
  formatfile=$2

  # 如果没有指定文件名或 .clang-format 文件，则提示用法
  if [ -z "$filename" ] || [ -z "$formatfile" ]; then
    error "用法：./format.sh <文件名> <.clang-format文件名>"
    exit 1
  fi

  # 检查文件是否存在
  if [ ! -f "$filename" ]; then
    error "错误：文件 $filename 不存在"
    exit 1
  fi

  # 检查 .clang-format 文件是否存在
  if [ ! -f "$formatfile" ]; then
    error "错误：文件 $formatfile 不存在"
    exit 1
  fi

  # 使用 Clang-Format 格式化文件
  clang-format -style=file -i "$filename"

  # 打印操作完成的消息
  ok "文件 $filename 已被格式化"
}

main $@
###############################################################################