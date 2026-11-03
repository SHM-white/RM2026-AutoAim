# 串口断线自动恢复与固定设备路径

## 结论

**关闭应用程序的文件描述符并不能保证下次 USB 枚举的 `/dev/ttyUSB0` 仍属于同一设备。** 这个编号由内核动态分配。程序必须同时做到：

1. 统一使用**稳定的设备路径**（udev symlink）。
2. 设备消失、文件节点变化、I/O 异常后关闭旧连接；重插后按**同一个稳定路径**重新打开。不能扫 `ttyUSB*` 然后连接第一个能打开的设备。

`io/reconnecting_serial.hpp` 负责第 2 步。通过 `stat()` 比较当前 symlink 目标的文件系统设备号、inode 和字符设备号，避免在同一名称重新分配给其他 tty 节点后继续使用旧 FD。读写异常及短写也会触发关闭。每 250ms 允许一次重试，但实际重试在下一次读/写调用时执行。

## 实机查找稳定设备名

把云台、导航和 IMU 三个设备都插上，在 Jetson/Ubuntu 上运行：

```bash
ls -l /dev/serial/by-id/ /dev/serial/by-path/
for d in /dev/ttyUSB* /dev/ttyACM*; do
  [ -e "$d" ] || continue
  echo "===== $d ====="
  udevadm info --query=property --name="$d" | grep -E '^(ID_SERIAL|ID_SERIAL_SHORT|ID_PATH|ID_VENDOR_ID|ID_MODEL_ID)='
done
```

**首选 `/dev/serial/by-id/`**：设备确有唯一的 USB 序列号时，可跟随同一设备跨 USB 接口/端口重插。**如没有唯一序列号，用 `/dev/serial/by-path/`**：绑定到固定物理 USB 口；换 USB 口或改变 Hub 拓扑将改变路径。

例如（下面是格式示意，不是真实设备名），将原有 `configs/sentry.yaml` 中两个字段替换为：

```yaml
com_port: "/dev/serial/by-id/usb-MANUFACTURER_GIMBAL_SERIAL-if00"
nav_com_port: "/dev/serial/by-id/usb-MANUFACTURER_NAV_SERIAL-if00"
```

IMU 目前没有配置文件入参，允许通过显式构造参数或环境变量提供稳定路径：

```bash
export SKYWALKER_IMU_PORT="/dev/serial/by-id/usb-MANUFACTURER_IMU_SERIAL-if00"
```

**不要同时用 `ttyUSB0`/`ttyUSB1` 作为正式部署路径**。无法通过关闭串口让动态编号固定。重插后可以用 `readlink -f /dev/serial/by-id/你的设备` 确认它实际指向哪个 `ttyUSBn`。

## 无唯一 USB 序列号时的自定义别名

如果设备固件都上报相同 USB 序列号，单纯按 VID/PID 匹配可能会把云台、导航分错。建议先用 `/dev/serial/by-path` 绑定物理 USB 口；需要友好的名称时，可以用精确的 `ID_PATH` 建立 symlink。

```bash
udevadm info --query=property --name=/dev/ttyUSB0 | grep '^ID_PATH='
```

根据实机结果编辑 `/etc/udev/rules.d/90-skywalker-serial.rules`，**将占位值替换为实际且互不重复的 `ID_PATH`**：

```udev
SUBSYSTEM=="tty", ENV{ID_PATH}=="REPLACE_WITH_GIMBAL_ID_PATH", SYMLINK+="skywalker/gimbal"
SUBSYSTEM=="tty", ENV{ID_PATH}=="REPLACE_WITH_NAV_ID_PATH", SYMLINK+="skywalker/nav"
```

应用：

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=tty
ls -l /dev/skywalker/
```

然后把 `com_port` 改为 `/dev/skywalker/gimbal`，`nav_com_port` 改为 `/dev/skywalker/nav`。如果多个设备插在同一 USB hub 上，务必逐个插拔验证对应关系。

## 断线行为

| 模块 | 自动恢复 | 控制行为 |
|---|---|---|
| `Gimbal` | 独立接收线程重试固定路径 | 姿态无效时不使用旧数据，失去新鲜反馈时发送空闲/不开火帧（仅端口仍可写时） |
| `NavSerial` | 每次导航发送时检查并尝试重新打开 | 失败返回 false，不缓存过期速度 |
| `DM_IMU` | 独立接收线程重试固定路径，重连后重新初始化 | 不阻塞初始化或姿态查询；重连不重复执行角度置零 |

注意：当前逻辑**不负责板端电机急停**。MCU 必须实现独立的串口通信超时保护与运动停止策略。务必分别测试拔掉云台线、导航线、IMU 线，以及 USB hub 断开后重连。

## 需要实测

1. 用原设备验证 USB 重新枚举后旧句柄能关闭，新设备节点能被自动打开，尤其是弹丸发射使能状态。
2. 在 USB hub/高负载下长时间热拔插，观察端口丢包和重连间隔。
3. 在 `sudo journalctl -k -f` 中确认每次拔插出现的 `ttyUSBn` 变化不影响稳定别名。
