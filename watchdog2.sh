#!/bin/bash

# =========================================================
# RM2026-AutoAim 守护进程脚本 (Watchdog)
# 如果程序崩溃或意外退出，该脚本会自动将其重新拉起
# =========================================================

# 指定要运行的目标程序列表 (请根据实际情况添加或修改)
TARGET_PROGRAMS=(
    "/home/xtyf/RM2026-AutoAim/build/sentry"
    # "./build/publish_test"
    # "./build/other_program"
)

# 确保在脚本退出时清理所有后台任务
trap "echo '[Watchdog] 正在关闭所有子进程...'; kill $(jobs -p) 2>/dev/null; exit" SIGINT SIGTERM

echo "[Watchdog] 守护进程已启动, 目标程序数量: ${#TARGET_PROGRAMS[@]}"

# 循环遍历每个目标程序并放到后台执行
for TARGET_PROGRAM in "${TARGET_PROGRAMS[@]}"; do
    (
        # 检查目标程序是否存在
        if [ ! -f "$TARGET_PROGRAM" ]; then
            echo "[Watchdog] 错误: 找不到目标程序 $TARGET_PROGRAM !"
            echo "[Watchdog] 请检查路径或确保代码已成功编译。"
            exit 1
        fi

        # 无限循环，实现断线/崩溃重连
        while true; do
            echo "========================================================="
            echo "[Watchdog] $(date '+%Y-%m-%d %H:%M:%S') 正在启动程序: $TARGET_PROGRAM"
            echo "========================================================="
            
            source /home/xtyf/.bashrc  # 加载环境变量（如果需要）
            # 运行目标程序
            $TARGET_PROGRAM
            
            # 程序退出后获取退出码
            EXIT_CODE=$?
            
            echo "========================================================="
            echo "[Watchdog] $(date '+%Y-%m-%d %H:%M:%S') 程序 $TARGET_PROGRAM 已退出，退出码: $EXIT_CODE"
            echo "[Watchdog] 3秒后尝试重新拉起程序: $TARGET_PROGRAM ..."
            echo "========================================================="
            
            # 防止因秒退而导致的CPU满载（死循环过快），设置适当的冷却时间
            sleep 3
        done
    ) &
done

# 等待所有后台子进程
wait
