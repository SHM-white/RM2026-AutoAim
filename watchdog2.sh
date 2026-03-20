#!/bin/bash

# =========================================================
# RM2026-AutoAim 守护进程脚本 (Watchdog)
# 如果程序崩溃或意外退出，该脚本会自动将其重新拉起
# =========================================================

# 指定要运行的目标程序路径 (请根据实际情况修改，比如 ./build/standard 或 ./build/auto_aim_test)
TARGET_PROGRAM="./build/auto_aim_test_new"

# 检查目标程序是否存在
if [ ! -f "$TARGET_PROGRAM" ]; then
    echo "[Watchdog] 错误: 找不到目标程序 $TARGET_PROGRAM !"
    echo "[Watchdog] 请检查路径或确保代码已成功编译。"
    echo "[Watchdog] 5秒后退出 watchdog..."
    sleep 5
    exit 1
fi

echo "[Watchdog] 守护进程已启动, 目标程序: $TARGET_PROGRAM"

# 无限循环，实现断线/崩溃重连
while true; do
    echo "========================================================="
    echo "[Watchdog] $(date '+%Y-%m-%d %H:%M:%S') 正在启动程序..."
    echo "========================================================="
    
    # 运行目标程序
    $TARGET_PROGRAM
    
    # 程序退出后获取退出码
    EXIT_CODE=$?
    
    echo "========================================================="
    echo "[Watchdog] $(date '+%Y-%m-%d %H:%M:%S') 程序已退出，退出码: $EXIT_CODE"
    echo "[Watchdog] 3秒后尝试重新拉起程序..."
    echo "========================================================="
    
    # 防止因秒退而导致的CPU满载（死循环过快），设置适当的冷却时间
    sleep 3
done
