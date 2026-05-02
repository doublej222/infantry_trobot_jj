## TRobot (DJI Development Board Type C)

**T**GU **R**obot: The new robot embedded development framework.

# RoboMaster Infantry TGU304
![Build Status](https://img.shields.io/badge/Build-CLion-blue)
![Platform](https://img.shields.io/badge/Platform-STM32F4-brightgreen)


## 项目简介

> 本项目为 ***RoboMaster Infantry*** 控制代码，采用大疆 C 型开发板。
>
> 🚧 学习中，控制代码仍在完善...

## 软件架构

### 核心任务模块 (`app/task/`)

系统通过 FreeRTOS 进行多任务调度，各任务独立运行：

| 任务文件 | 功能描述 |
| :--- | :--- |
| **`chassis.cc`** | 底盘运动控制：麦轮解算、底盘模式切换 |
| **`gimbal.cc`** | 云台控制：Yaw 轴 PID 计算、角度闭环控制 |
| **`msg_task.cc`** | Gimbal 与 Chassis 之间的消息通信 |

### 文件结构

```text
trobot_f4_gimbal/
├── app/                      # 应用层
│   ├── main.cc               # 初始化
│   └── task/                 # 任务模块
│       ├── chassis.cc        # 底盘控制任务
│       ├── gimbal.cc         # 云台控制任务
│       └── msg_task.cc       # 消息处理任务
├── bsp/                      # 板级支持包
└──components/                # 组件
