#!/bin/bash
sleep 5
cd /home/xtyf/RM2026-AutoAim/
screen \
    -S autoaim \
    -L \
    -Logfile logs/$(date "+%Y-%m-%d_%H-%M-%S").screenlog \
    -d \
    -m \
    bash -c "./watchdog.sh"
