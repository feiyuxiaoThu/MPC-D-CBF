#!/bin/bash

rosnode kill -a
# 查找所有 gzserver 进程，并按 CPU 使用率排序，选择占用最高的一个
PID=$(ps aux | grep 'gzserver' | grep -v 'grep' | sort -nrk 3,3 | head -n 1 | awk '{print $2}')

# 如果找到进程，则杀死它
if [ -n "$PID" ]; then
    echo "正在杀死 PID 为 $PID 的 gzserver 进程..."
    kill -9 $PID
    echo "进程 $PID 已被终止。"
else
    echo "没有找到 gzserver 进程。"
fi
