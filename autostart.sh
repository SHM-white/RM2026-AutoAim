#!/bin/bash
sleep 5
cd /home/xtyf/RM2026-AutoAim/
source /opt/ros/humble/setup.bash
screen \
    -S autoaim \
    -L \
    -Logfile logs/$(date "+%Y-%m-%d_%H-%M-%S").screenlog \
    -d \
    -m \
    bash -c "./watchdog.sh"
