#!/bin/bash

#awk 判断 ps 命令输出的第 8 列为 Z 是,显示该进程的 PID 和进程命令
ps aux |awk '{if($8 == "Z"){print $2,$11}}'
