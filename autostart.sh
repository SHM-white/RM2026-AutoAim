#!/bin/bash
sleep 5
cd /home/xtyf/RM2026-AutoAim/
source /opt/ros/humble/setup.bash

run_cmd="cd /home/xtyf/RM2026-AutoAim && source /opt/ros/humble/setup.bash && ./watchdog.sh"

# 默认弹出可见终端，便于随时手动停止。
if [ -n "$DISPLAY" ]; then
    if command -v gnome-terminal >/dev/null 2>&1; then
        gnome-terminal -- bash -lc "$run_cmd; exec bash"
    elif command -v xfce4-terminal >/dev/null 2>&1; then
        xfce4-terminal --hold -e "bash -lc '$run_cmd'"
    elif command -v konsole >/dev/null 2>&1; then
        konsole --hold -e bash -lc "$run_cmd"
    elif command -v x-terminal-emulator >/dev/null 2>&1; then
        x-terminal-emulator -e bash -lc "$run_cmd"
    else
        screen \
            -S autoaim \
            -L \
            -Logfile logs/$(date "+%Y-%m-%d_%H-%M-%S").screenlog \
            -d \
            -m \
            bash -lc "$run_cmd"
    fi
else
    screen \
        -S autoaim \
        -L \
        -Logfile logs/$(date "+%Y-%m-%d_%H-%M-%S").screenlog \
        -d \
        -m \
        bash -lc "$run_cmd"
fi
