# CARLA 场景编辑器与运行器

本项目包含两个核心工具：**`waypoints.py`**（场景编辑器）和 **`run.py`**（场景运行器），用于在 CARLA 仿真环境中快速设计、生成和运行自动驾驶测试场景。

---

## 目录

1. [前置准备](#前置准备)
2. [waypoints.py — 场景编辑器](#waypointspy--场景编辑器)
3. [run.py — 场景运行器](#runpy--场景运行器)
4. [场景速查表](#场景速查表)

---

## 前置准备

1. 启动 CARLA 服务器：
   ```bash
   # Linux
   ./CarlaUE4.sh -windowed -benchmark -fps=20
   # Windows
   CarlaUE4.exe -windowed -benchmark -fps=20
   ```
   默认监听 `127.0.0.1:2000`。

2. 安装依赖：
   ```bash
   pip install carla pygame numpy imageio opencv-python pandas
   ```

3. （可选）如果使用 TCP 模型控制，需要准备 `tcp/best_model.ckpt`。

---

## waypoints.py — 场景编辑器

交互式地图编辑器，用于在 CARLA 地图上绘制场景要素并生成 JSON 配置文件。

### 启动方式

```bash
# 默认参数（连接 localhost:2000，地图 TOWN10HD_Opt）
python waypoints.py

# 自定义参数
python waypoints.py --host 127.0.0.1 --port 2000 --name TOWN10HD_Opt --scenario 1 --save_dir ./save_scenarios
```

| 参数 | 说明 | 默认值 |
|---|---|---|
| `--host` | CARLA 服务器 IP | `127.0.0.1` |
| `--port` | CARLA 服务器端口 | `2000` |
| `--name` | CARLA 地图名称 | `TOWN10HD_Opt` |
| `--scenario` | 场景编号（用于文件名） | `1` |
| `--save_dir` | JSON 输出目录 | `output` |

### 界面操作

编辑器打开后显示 CARLA 地图的俯视图：

```
┌──────────────────────────────────────────┐
│                                          │
│         CARLA 地图俯视图                   │
│    (灰色道路 · 白色路点 · 绿色朝向箭头)     │
│                                          │
├──────────────────────────────────────────┤
│  使用说明:                                 │
│  中键拖拽=平移  滚轮=缩放  Q=退出          │
│  画完后按 S 自动生成 64 个天气场景           │
└──────────────────────────────────────────┘
```

### 核心操作步骤

#### Step 1：设置 Ego 车辆起始点（必选）

- **操作**：按住 `Ctrl` + 鼠标左键点击地图
- **效果**：出现蓝色圆圈标注 **"EGO"**
- Ego 车辆自动吸附到最近车道，z 轴比地面高 0.3m

![Trigger + Ego + Agent 1 位置示意](docs/scenarios/01_trigger_ego.png)

#### Step 2：设置 Trigger 触发点（必选）

- **操作**：按住 `Shift` + 鼠标左键点击地图
- **效果**：出现红色圆圈标注 **"TRIGGER"**
- 当 Ego 车辆进入触发点附近（< 10m）时，场景逻辑开始执行

```
              Shift + 左键
                    ↓
                  ◉ TRIGGER ← 红色圆点
```

#### Step 3：添加 Agent 智能体（可选）

- **操作**：鼠标右键点击地图
- **效果**：添加默认类型的智能体（蓝色圆圈）

![Agent 智能体类型一览](docs/scenarios/02_agents.png)

| 按键 | 功能 |
|---|---|
| `5` | 切换为 **行人** (walker) |
| `6` | 切换为 **自行车** (bike) |
| `7` | 切换为 **汽车** (car) |
| `8` | 切换为 **障碍物** (obstacle) |

#### 选中与编辑智能体

1. **选中**：鼠标左键点击智能体 → 变为黄色高亮
2. **调整朝向**（选中状态下）：

| 按键 | 朝向 |
|---|---|
| `1` | 沿车道正向 |
| `2` | 沿车道反向（180°） |
| `3` | 车道左侧（-90°） |
| `4` | 车道右侧（+90°） |

3. **删除**：选中后按 `Delete` 键

#### Step 4：（可选）绘制 Ego 循迹路线

- **操作**：按住 `Alt` + 鼠标右键点击地图
- **效果**：添加绿色路线点 P1, P2, ...，用于 EgoRouteFollow 场景

```
  Alt + 右键     Alt + 右键
       ↓              ↓
      ●P1 ────●P2 ────●P3  ← 绿色连线
```

- 按 `C` 键清空所有路线点

#### Step 5：保存场景

- **操作**：按 `S` 键
- **效果**：自动生成 **64 个 JSON 文件**，覆盖 8 大类 × 8 种强度的天气组合

天气类型包括：晴天、多云、阴天、小雨、大雨、大雾、大风、沙尘暴，以及夜晚、黄昏、黎明等。

```
save_scenarios/
├── scenario_1_0000_sunny_01.json
├── scenario_1_0000_sunny_02.json
├── ...
├── scenario_1_0000_fog_08.json
├── scenario_1_0000_night_01.json
└── ... (共 64 个)
```

#### Step 6：退出

- 按 `Q` 键关闭窗口

### 导航操作

| 操作 | 功能 |
|---|---|
| 鼠标中键拖拽 | 平移地图 |
| 鼠标滚轮 | 缩放地图 |

---

## run.py — 场景运行器

遍历 `save_scenarios/` 目录下所有场景 JSON 文件，依次在 CARLA 中运行并录制视频。

### 启动方式

```bash
# 运行指定场景类型（默认 behavior 模式，使用 CARLA 自动驾驶）
python run.py --input_dir ./save_scenarios/ --town TOWN10HD_Opt --scenario 3a

# 运行指定场景，使用 TCP 模型控制
python run.py --input_dir ./save_scenarios/ --town TOWN10HD_Opt --scenario 3a --model tcp --model_path ./tcp/best_model.ckpt

# 切换地图和场景
python run.py --input_dir ./save_scenarios/ --town center --scenario 2b
```

| 参数 | 说明 | 默认值 |
|---|---|---|
| `--town` | CARLA 地图名称 | 必填 |
| `--route_id` | 路由 ID | `route_01` |
| `--input_dir` | 场景 JSON 文件目录 | 必填 |
| `--scenario` | 场景类型编码 | `3a` |
| `--video_dir` | 视频输出根目录 | `videos` |
| `--model` | 控制模式：`behavior` / `tcp` | `behavior` |
| `--model_path` | TCP 模型权重路径 | `./tcp/best_model.ckpt` |

### 运行流程

```
输入目录 (JSON)
       │
       ▼
┌─────────────┐
│ 加载天气参数  │ ← 从 JSON 自动读取云量/降雨/雾等
└──────┬──────┘
       ▼
┌─────────────┐
│ 生成 Ego 车辆 │ ← 按 ego_start 配置 spawn
└──────┬──────┘
       ▼
┌─────────────┐
│ 生成 Agent    │ ← 行人/车辆/障碍物
└──────┬──────┘
       ▼
┌─────────────┐
│ TCP 或 autopilot │ ← 控制 Ego 行驶
└──────┬──────┘
       ▼
┌─────────────┐
│ 触发场景逻辑  │ ← Ego 到达 trigger 点后激活
└──────┬──────┘
       ▼
┌─────────────┐
│ 录制视频     │ ← 每场景输出一个 MP4
└──────┬──────┘
       ▼
┌─────────────┐
│ 保存结果     │ ← 碰撞/速度等数据存入 result.pkl
└─────────────┘
```

### 实时可视化界面

运行时会弹出一个窗口，左侧为鸟瞰图（BEV），右侧为前视相机画面：

```
┌────────────────────┬────────────────────┐
│                    │                    │
│   鸟瞰图 (BEV)      │   前视相机           │
│                    │                    │
│    ┌───┐           │   ╔═══════╗         │
│    │ 🚗│ Ego 车辆    │   ║ 前方  ║         │
│    └───┘           │   ║ 视野  ║         │
│   ↕ 蓝色轨迹        │   ╚═══════╝         │
│   其他车辆          │                    │
│   其他行人          │                    │
│                    │                    │
└────────────────────┴────────────────────┘
```

- **左侧 BEV**：俯视视角，显示道路、Ego 车辆（青色）、其他车辆（绿色）、行人（蓝色）
- **右侧相机**：Ego 车载 RGB 相机实时画面

### 输出文件

```
videos/
├── 3a/                          # 按 --scenario 分文件夹
│   ├── scenario_1_0000_sunny_01.mp4
│   ├── scenario_1_0000_sunny_02.mp4
│   └── ...
│   └── 3a_result.pkl            # 结果数据（碰撞/速度/距离等）
```

---

## 场景速查表

`--scenario` 参数决定场景类型，每种场景对应不同的 `scene.py` 类。以下按 Excel `1.xlsx` 中"测试场景"列分类整理：

### 1. 交通信号识别及响应

| 场景编码 | 测试场景（Excel） | 场景类名 | 示意图 |
|---------|-----------------|---------|--------|
| **1.a** | 限速标志 | - | ![1a](docs/scenarios/02a_speedlimit.png) |
| **1.b** | 弯道 | - | ![1b](docs/scenarios/01b_curve.png) |

### 2. 道路交通基础设施与障碍物识别及响应

| 场景编码 | 测试场景（Excel） | 场景类名 | 示意图 |
|---------|-----------------|---------|--------|
| **2.b** | 环形路口 | `EgoRouteFollowScene` | ![2b](docs/scenarios/2b_roundabout.png) |
| **2.c** | 无信号灯路口左侧存在直行车辆 | `CarCrossScene` | ![2c](docs/scenarios/2c_intersection_left.png) |
| **2.d** | 无信号灯路口右侧存在直行车辆 | `CarCrossScene` | ![2d](docs/scenarios/2d_intersection_right.png) |
| **2.e** | 无信号灯路口对向存在直行车辆 | `CarCrossScene` | ![2e](docs/scenarios/2e_intersection_oncoming.png) |
| **2.f** | 施工车道 | `StaticObstacleScene` | ![2f](docs/scenarios/2f_construction.png) |
| **2.g** | 静止车辆占用部分车道 | `StaticCarCrossScene` | ![2g](docs/scenarios/2g_parked_vehicle.png) |

### 3. 周边车辆行驶状态识别及响应

| 场景编码 | 测试场景（Excel） | 场景类名 | 示意图 |
|---------|-----------------|---------|--------|
| **3.a** | 行人通过人行横道线(1) | `PedestrianCrossScene` | ![3a](docs/scenarios/3a_crosswalk.png) |
| **3.b I** | 行人沿道路行走Ⅰ（1） | `PedestrianCrossScene` | ![3b1](docs/scenarios/3b1_walk_parallel1.png) |
| **3.b II** | 行人沿道路行走Ⅱ（1） | `PedestrianCrossScene` | ![3b2](docs/scenarios/3b2_walk_parallel2.png) |
| **3.c** | 自行车同车道骑行（1） | `BicycleCrossScene` | ![3c](docs/scenarios/3c_bike_same_lane.png) |
| **3.d** | 行人目标感知受阻（1） | `OccludedPedestrianScene` | ![3d](docs/scenarios/3d_occluded.png) |
| **4.a** | 前方车辆切入 | `CarCutInScene` | ![4a](docs/scenarios/4a_cut_in.png) |
| **4.b** | 前方车辆切出 | `CarCutOutScene` | ![4b](docs/scenarios/4b_cut_out.png) |
| **4.c** | 对向车辆借道行驶(1) | `CarOncomingPassScene` | ![4c](docs/scenarios/4c_overtake.png) |
| **4.d** | 目标车辆停-走 | `CarStopandGoScene` | ![4d](docs/scenarios/4d_stop_go.png) |

### 5. 自动紧急避险

| 场景编码 | 测试场景（Excel） | 场景类名 | 示意图 |
|---------|-----------------|---------|--------|
| **5.a** | 行人横穿道路 | `PedestrianCrossScene` | ![5a](docs/scenarios/5a_pedestrian_emergency.png) |
| **5.b** | 自行车横穿道路 | `BicycleCrossScene` | ![5b](docs/scenarios/5b_bike_emergency.png) |
| **5.c** | 目标车辆切出后存在静止车辆 | `CarCutOutandStaticScene` | ![5c](docs/scenarios/5c_cutout_then_static.png) |
| **5.d** | 前方车辆紧急制动 | `CarGoandStopScene` | ![5d](docs/scenarios/5d_emergency_brake.png) |
| **5.e** | 紧急转弯危险情况 | `CarCrossScene` | ![5e](docs/scenarios/5e_emergency_swerve.png) |
| **5.f** | 静止行人目标误触发 | `StaticPedestrianCrossScene` | ![5f](docs/scenarios/5f_static_ped_false.png) |
| **5.g** | 移动行人目标误触发 | `PedestrianCrossScene` | ![5g](docs/scenarios/5g_moving_ped_false.png) |

### 6. 停车

| 场景编码 | 测试场景（Excel） | 场景类名 | 示意图 |
|---------|-----------------|---------|--------|
| **6.a** | 停车点 | `EgoRouteFollowScene` | ![6a](docs/scenarios/6a_parking_spot.png) |
| **6.b** | 港湾式站台 | `EgoRouteFollowScene` | ![6b](docs/scenarios/6b_bus_bay.png) |
| **6.c** | 普通站台 | `EgoRouteFollowScene` | ![6c](docs/scenarios/6c_regular_bus_stop.png) |

### 场景详细说明

#### 1.a — 限速标志

道路旁设置限速标志，Ego 车辆需要识别并响应。

![限速标志](docs/scenarios/02a_speedlimit.png)

#### 1.b — 弯道

Ego 车辆在弯道上行驶，需要正确跟踪车道曲线。

![弯道](docs/scenarios/01b_curve.png)

#### 2.b — 环形路口

Ego 车辆进入环形路口，需处理多方向汇入车辆。

![环形路口](docs/scenarios/2b_roundabout.png)

#### 2.c — 无信号灯路口左侧存在直行车辆

十字路口，左侧道路有车辆直行通过路口。

![路口左侧直行](docs/scenarios/2c_intersection_left.png)

#### 2.d — 无信号灯路口右侧存在直行车辆

十字路口，右侧道路有车辆直行通过路口。

![路口右侧直行](docs/scenarios/2d_intersection_right.png)

#### 2.e — 无信号灯路口对向存在直行车辆

十字路口，对向车道有车辆直行通过路口。

![路口对向直行](docs/scenarios/2e_intersection_oncoming.png)

#### 2.f — 施工车道

道路前方出现施工锥桶封锁部分车道，Ego 需绕行。

![施工车道](docs/scenarios/2f_construction.png)

#### 2.g — 静止车辆占用部分车道

一辆车停在路边，部分占据行车道。

![静止车辆占车道](docs/scenarios/2g_parked_vehicle.png)

#### 3.a — 行人通过人行横道线

Ego 车辆行驶至人行横道，行人（Agent 1/2）正在横穿马路。

![行人通过人行横道线](docs/scenarios/3a_crosswalk.png)

#### 3.b I — 行人沿道路行走Ⅰ

行人与 Ego 同向沿道路行走，测试纵向相对运动识别。

![行人沿道路行走 I](docs/scenarios/3b1_walk_parallel1.png)

#### 3.b II — 行人沿道路行走Ⅱ

行人在平行道路上反向行走，测试不同相对速度下的识别。

![行人沿道路行走 II](docs/scenarios/3b2_walk_parallel2.png)

#### 3.c — 自行车同车道骑行

自行车与 Ego 在同一条车道上同向行驶。

![自行车同车道骑行](docs/scenarios/3c_bike_same_lane.png)

#### 3.d — 行人目标感知受阻

行人在容器（障碍物）后方被遮挡，突然走出测试感知系统的遮挡处理能力。

![行人目标感知受阻](docs/scenarios/3d_occluded.png)

#### 4.a — 前方车辆切入

前方车辆从相邻车道变道切入 Ego 前方，Ego 需减速避让。

![前方车辆切入](docs/scenarios/4a_cut_in.png)

#### 4.b — 前方车辆切出

Ego 前方车辆突然向侧方变道离开，可能暴露前方障碍物。

![前方车辆切出](docs/scenarios/4b_cut_out.png)

#### 4.c — 对向车辆借道行驶

对向来车借用 Ego 所在车道超车，Ego 需减速让行。

![对向车辆借道行驶](docs/scenarios/4c_overtake.png)

#### 4.d — 目标车辆停-走

前方车辆先静止 2 秒，然后起步加速，测试 Ego 的跟停-跟启能力。

![目标车辆停-走](docs/scenarios/4d_stop_go.png)

#### 5.a — 行人横穿道路（紧急避险）

行人突然横穿道路，Ego 需要紧急制动。

![行人横穿道路](docs/scenarios/5a_pedestrian_emergency.png)

#### 5.b — 自行车横穿道路（紧急避险）

自行车突然横穿道路，Ego 需要紧急避让。

![自行车横穿道路](docs/scenarios/5b_bike_emergency.png)

#### 5.c — 目标车辆切出后存在静止车辆

前车切出后，暴露出后方静止车辆，Ego 需及时制动。

![切出后存在静止车辆](docs/scenarios/5c_cutout_then_static.png)

#### 5.d — 前方车辆紧急制动

前方车辆正常行驶时突然紧急制动，Ego 需迅速反应。

![前方车辆紧急制动](docs/scenarios/5d_emergency_brake.png)

#### 5.e — 紧急转弯危险情况

前方出现障碍物，Ego 需要紧急转弯避让。

![紧急转弯危险情况](docs/scenarios/5e_emergency_swerve.png)

#### 5.f — 静止行人目标误触发

静止不动的行人，测试系统是否会误触发制动/转向。

![静止行人目标误触发](docs/scenarios/5f_static_ped_false.png)

#### 5.g — 移动行人目标误触发

横向移动的行人，测试系统对运动方向判断的准确性。

![移动行人目标误触发](docs/scenarios/5g_moving_ped_false.png)

#### 6.a — 停车点

Ego 车辆按路线行驶至指定停车点并停靠。

![停车点](docs/scenarios/6a_parking_spot.png)

#### 6.b — 港湾式站台

Ego 车辆驶入港湾式公交站台停靠。

![港湾式站台](docs/scenarios/6b_bus_bay.png)

#### 6.c — 普通站台

Ego 车辆在普通路边站台停靠，旁边有等车行人（Agent 1）。

![普通站台](docs/scenarios/6c_regular_bus_stop.png)
