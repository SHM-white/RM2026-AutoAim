# 2027 视觉迁移说明

> 当前分支是**接口迁移和串口隔离**，不是已完成 2027 赛季靶标识别的可实战版本。

## 新赛季特征：只换函数，不增配置

- 识别：`tasks/auto_aim/yolo.cpp` 的 `create_season_detector()` 是唯一的型号接入入口。新增模型时实现 `YOLOBase::detect()`，返回的 `Armor::points` 必须是可靠的四个物理特征点（TL/TR/BR/BL），不能直接把检测框的四角塞给 PnP。
- PnP：`tasks/auto_aim/armor_geometry.hpp` 的 `armor_object_points(type, name)` 是唯一 3D 几何替换点。当前 **仍是 2026 灯条端点**，正式接入新赛季点阵屏/特征后，替换这个函数并保证 2D/3D 点位一一对应。
- 保留原 YOLOv5/v8/11 OpenVINO 模型兼容路径，没有新增新赛季的 YAML 规则或注册表。
- YOLOE/文本提示只预留虚接口；文本提示目标检测不等于关键点定位，不能未经测试直接给 PnP。
- Jetson Xavier NX 的 CUDA/TensorRT 推理执行器 **尚未实现与测试**。需依实际 JetPack/TensorRT 版本适配模型和输出解码。

## 通讯变化

- `Gimbal`：仍使用 `com_port`，只负责云台收发及其原有裁判系统接收。
- `NavSerial`：只使用独立的 `nav_com_port`，负责导航控制。**不能与 `com_port` 相同**。
- 共用 `io/vision_protocol.hpp` 的 AB 帧头、打包结构及 CRC16；导航帧的旧 CD 帧头不再发送。
- 新协议的 `NavData` 仍使用六个 little-endian IEEE-754 double，大小 55 字节。MCU/导航端必须同步更新帧头/结构、校验算法和串口路由。
- 自瞄 Gimbal TX/RX 和导航 TX 不是同一帧长度；靠**物理串口**区分，不应在一条线混用。
- 新的 sentry 入口不再调用通过云台转发导航和 IMU 的旧方法。其它历史示例可能仍有单路 IMU 转发接口，未对其 MCU 兼容性作承诺。

## 清理策略

- CBoard/CBoardUART/SocketCAN 和相关实验入口不在默认构建中。
- 只在显式开启 `-DENABLE_LEGACY_CBOARD=ON` 时尝试构建旧版 CAN 实验目标。它们暂时保留源码供迁移核对，并非新赛季运行依赖。

## 验证清单

1. 用本仓库自带测试验证 PnP 静态点位顺序和导航封包 CRC16。
2. 在实际 Jetson 和导航/云台 MCU 上分别测试两路串口，特别检查数据帧头、流速、断线超时与急停。
3. 新赛季视觉需要真实标定点、图像样本、标注与物理尺寸；不能直接按旧赛季模型上场。
4. 重新测试识别延迟、PnP 重投影误差、目标跟踪以及通信失联时禁止射击/导航归零。
