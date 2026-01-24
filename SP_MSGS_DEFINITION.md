# sp_msgs 消息定义整理

## 项目结构

```
sp_msgs/
├── CMakeLists.txt
├── package.xml
└── msg/
    ├── EnemyStatusMsg.msg
    └── AutoaimTargetMsg.msg
```

## 消息定义详情

### 1. EnemyStatusMsg.msg

**用途**: 敌人状态信息，包含不可破坏敌人的ID列表

**字段定义**:
```
builtin_interfaces/Time timestamp  # 消息时间戳
int8[] invincible_enemy_ids        # 不可破坏敌人的ID数组
```

**使用位置**:
- [io/ros2/subscribe2nav.cpp](io/ros2/subscribe2nav.cpp#L16) - 订阅敌人状态
- [tests/topic_loop_test.cpp](tests/topic_loop_test.cpp#L14) - 测试发布敌人状态

**访问字段示例**:
```cpp
sp_msgs::msg::EnemyStatusMsg msg;
msg.timestamp.sec      // 秒
msg.timestamp.nanosec  // 纳秒
msg.invincible_enemy_ids  // 敌人ID向量
```

### 2. AutoaimTargetMsg.msg

**用途**: 自瞄目标信息，包含目标ID列表

**字段定义**:
```
builtin_interfaces/Time timestamp  # 消息时间戳
int8[] target_ids                  # 目标ID数组
```

**使用位置**:
- [io/ros2/subscribe2nav.cpp](io/ros2/subscribe2nav.cpp#L20) - 订阅自瞄目标

**访问字段示例**:
```cpp
sp_msgs::msg::AutoaimTargetMsg msg;
msg.timestamp.sec      // 秒
msg.timestamp.nanosec  // 纳秒
msg.target_ids         // 目标ID向量
```

## 编译和使用

### 构建流程

1. **构建sp_msgs包** (在ROS2工作空间中):
```bash
colcon build --packages-select sp_msgs
```

2. **使用生成的消息**:
```cpp
#include "sp_msgs/msg/enemy_status_msg.hpp"
#include "sp_msgs/msg/autoaim_target_msg.hpp"

// 使用示例
sp_msgs::msg::EnemyStatusMsg status_msg;
sp_msgs::msg::AutoaimTargetMsg target_msg;
```

### 依赖关系

- **构建依赖**: ament_cmake, rosidl_cmake, builtin_interfaces
- **运行依赖**: builtin_interfaces, rosidl_default_runtime

## 关键特性

1. **时间戳支持**: 两个消息都使用ROS2的`builtin_interfaces/Time`类型
   - `sec`: 秒部分
   - `nanosec`: 纳秒部分

2. **动态数组**: 两个消息都使用`int8[]`数组类型存储ID
   - 可以包含任意数量的ID
   - 自动序列化/反序列化

3. **自动代码生成**: rosidl会自动生成:
   - C++类定义
   - 序列化/反序列化代码
   - 类型支持库

## 集成说明

sp_msgs包已添加到主项目的CMakeLists.txt中，作为首个构建子目录，以确保其他模块可以正确引用这些消息定义。
