# OpenClaw-RDKX5 多模态跨楼层巡检系统

基于 **RDK X5 + ROS 2 + OpenClaw + Qwen3-VL** 的全自动工业仪表巡检与识别系统。

端侧双摄像头 **BPU 视觉闭环**（电梯门识别 / 楼层号识别 / 跨楼层换层确认），
云端 **VLM 仪表读数融合**（几何读数 × 量程），支持语音/文字指令一键触发巡检。

---

## ✨ 核心特性

- **端侧双 BPU YOLO 节点**（同一模型，双摄像头独立推理）
  - **former 节点**（前摄像头）：识别**电梯门开/关状态**，门开自动直行进/出电梯
  - **back 节点**（后摄像头）：识别**楼层号**，视觉确认换层完成
- **跨楼层自主巡检**：多楼层地图自动切换，电梯换层全闭环
- **仪表自动识别**：导航到点 → 拍照 → AI 识别 → 记录 → 汇总，全程无人值守
- **端云协同读数**：端侧几何读数 × 云端 VLM 量程融合，检测失败自动降级
- **可视化仪表盘**：实时进度 + 多轮历史记录对比
- **聊天指令触发**：Web 控制台 / 飞书一句话开始、反向、停止巡检

## 系统架构

```
┌────────────────────────────────────────────────────┐
│                  OpenClaw（控制中枢）                │
│   run-inspection.sh → 编排巡检流程                  │
│   start/stop_inspection.sh → 启动/停止              │
│   check_progress.sh → 进度推送                      │
│   img-understand.sh → 调用视觉模型                  │
└───────┬───────────────────────┬────────────────────┘
        │ HTTP                  │ HTTP / ROS 2
        ▼                       ▼
┌─────────────────┐   ┌──────────────────────────────┐
│  VPS 服务端      │   │  RDK X5 小车端（本仓库）       │
│  Inspection     │   │  ┌────────────────────────┐  │
│  Server (Flask) │   │  │ former 节点（前摄）       │  │
│  :8001          │   │  │  BPU: 电梯门开/关识别     │  │
│  /upload        │   │  │  → 直行进/出电梯          │  │
│  /api/results   │   │  ├────────────────────────┤  │
│  /api/progress  │   │  │ back 节点（后摄）         │  │
│  dashboard.html │   │  │  BPU: 楼层号识别          │  │
│                 │   │  │  → 换层确认              │  │
│  Qwen3-VL 识别   │   │  ├────────────────────────┤  │
│  量程提取/读数    │   │  │ map_switch：Nav2 地图切换│  │
└─────────────────┘   │  │ go_straight：定距直行     │  │
                      │  │ openclaw_server：HTTP 网关│  │
                      │  └────────────────────────┘  │
                      └──────────────────────────────┘
```

## 核心闭环

### 1. 跨楼层巡检流程

```
开始巡检 → 机器人逐个导航到点位 → 拍照 → 上传服务器
→ AI 识别仪表 → 记录结果 → 导航到电梯口 → 换层
→ 继续巡检 → 全部完成
```

### 2. 电梯门识别与进出电梯（端侧 BPU，前摄）

```
former 节点订阅 /former_usbcam/image_raw
→ BPU 推理识别电梯门开/关
→ 门开 → 发布 go_straight 动作 → 直行进入电梯
→ 门未开 → 原地等待，不硬闯
```

### 3. 楼层号识别与换层确认（端侧 BPU，后摄）

```
openclaw_server 发布 /next_floor（目标楼层）
→ back 节点订阅 /back_usbcam/image_raw，BPU 识别楼层号
→ 匹配目标楼层（target_class）→ 发布 "F<floor>" 地图切换指令
→ map_switch_node 调用 Nav2 load_map 切换楼层地图
→ 发布 MAP_SWITCH_COMPLETED:<floor>
→ openclaw_server 回调 → POST /floor_changed 通知服务端
```

换层全程由端侧视觉确认，不依赖云端，**断网也能完成跨楼层切换**。

### 4. 仪表读数（端云协同）

```
导航到点 → 前摄拍照上传 → 云端 YOLO 检测表盘 4 关键点（Center/Start/End/Tip）
→ 几何校验通过 → 几何读数 × Qwen3-VL 提取量程 → 融合输出
→ 校验失败 → 直接采用 Qwen3-VL 识别结果
```

## 仓库结构

```
├── yolov5_detect/          # 端侧 BPU YOLO 检测（核心）
│   ├── yolo_detector/      #   former/back 节点 + BPU 推理封装
│   └── yolo_msgs/          #   检测结果消息定义
├── robot_elevator/         # 电梯交互
│   ├── go_straight/        #   定距直行 Action Server（进/出电梯）
│   └── elevator_interfaces/#   GoStraight 自定义 Action
├── stm32_map/              # 地图切换
│   ├── map_switch_node.cpp #   Nav2 楼层地图切换 + MAP_SWITCH 状态
│   └── maps/               #   各楼层栅格地图（lab 走廊）
├── openclaw_server/        # HTTP 网关（ROS 节点）
│   └── openclaw_server_node.py  # /nav/goal /floor_change /cmd/motion 等
├── camera_bringup/         # 前后摄像头启动
├── map_editor/             # PGM 地图坐标编辑工具（免 rviz2 取点）
└── README.md
```

## 部署指南

### 硬件要求

- **RDK X5** 开发者套件（主控，10 TOPS，BPU 加速）
- wheeltec 移动底盘 + 前后两路 USB 摄像头
- 电梯环境（多楼层场景，地图见 `stm32_map/maps/`）

### 依赖

- **ROS 2**（Humble，RDK OS / TogetheROS.Bot 环境）
- **Nav2** + **slam_toolbox**（导航与建图）
- **usb_cam**（摄像头驱动）
- **YOLO BPU 模型**：自训练模型（电梯门 / 楼层号），经地平线工具链转换后部署（模型文件体积较大，不随仓库分发）
- 云端：阿里云 VPS + Qwen3-VL 视觉模型 API

### 小车端构建与运行

```bash
# 1. 将仓库放到 ROS 2 工作空间
cd ~/wheeltec_ros2/src && git clone https://github.com/leolanqin/lanfanqie

# 2. 构建
cd ~/wheeltec_ros2 && colcon build --packages-select camera_bringup yolov5_detect yolo_msgs robot_elevator stm32_map openclaw_server

# 3. 启动摄像头（前/后）
ros2 launch camera_bringup demo1.launch.py

# 4. 启动 BPU 识别节点
ros2 launch yolo_detector former_camera_and_yolo_gostraight.launch.py
ros2 launch yolo_detector back_camera_and_yolo_gostraight.launch.py

# 5. 启动导航服务器与地图切换
ros2 run openclaw_server openclaw_server_node.py
ros2 run stm32_map map_switch_node
```

### 服务端（VPS）

巡检服务器（Flask）与巡检编排脚本正在整理中，将尽快开源；
接口约定：`POST /upload`（图片上传）、`POST /floor_changed`（楼层变更回调）、`GET /api/results`（结果查询）。
OpenClaw 部署参考：https://developer.aliyun.com/article/1710314

### 触发巡检

| 指令 | 功能 |
|------|------|
| `开始巡检` | 正向巡检（按 plan 顺序） |
| `倒回去` | 反向巡检（逆序） |
| `停下巡检` | 中断当前巡检 |

进度：Web 控制台实时推送 / 飞书完成后摘要 / 仪表盘历史对比。

## 第三方开源项目

- **[Pointer-Reading-based-on-YOLO](https://github.com/)**：指针式仪表 4 关键点检测（Center/Start/End/Tip）的训练与推理思路参考
- **YOLOv5 / YOLOv8**：检测模型基础
- **TogetheROS.Bot**：RDK 机器人中间件
- **Nav2 / slam_toolbox**：导航与建图
- **Qwen3-VL**：云端视觉语言模型
- **OpenClaw**：智能体控制中枢

## License

Apache 2.0
