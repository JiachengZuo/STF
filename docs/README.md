# CARLA 场景编辑器与运行器

本项目包含两个核心工具：**`waypoints.py`**（场景编辑器）和 **`run.py`**（场景运行器），用于在 CARLA 仿真环境中快速设计、生成和运行自动驾驶测试场景。

> 场景专项文档：[2.b 环形路口快速运行](scenarios/2b_roundabout/run_existing.md) ·
> [2.b 完整说明](scenarios/2b_roundabout/README.md) ·
> [全部场景索引](scenarios/README.md)
> [TCP权重链接](https://drive.google.com/drive/folders/19ZsHo_-98nbQcHDJL8_bEWX7xE-iHP6m?usp=sharing)

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

![Trigger + Ego ](1.png)

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

![Agent 智能体类型一览](2.png)

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
| **1.a** | 限速标志 | - | ![1a](3.png) |
| **1.b** | 弯道 | - | ![1b](4.png) |

### 2. 道路交通基础设施与障碍物识别及响应

| 场景编码 | 测试场景（Excel） | 场景类名 | 示意图 |
|---------|-----------------|---------|--------|
| **2.b** | 环形路口 | `EgoRouteFollowScene` | ![2b](5.png) |
| **2.c** | 无信号灯路口左侧存在直行车辆 | `CarCrossScene` | ![2c](6.png) |
| **2.d** | 无信号灯路口右侧存在直行车辆 | `CarCrossScene` | ![2d](7.png) |
| **2.e** | 无信号灯路口对向存在直行车辆 | `CarCrossScene` | ![2e](8.png) |
| **2.f** | 施工车道 | `StaticObstacleScene` | ![2f](9.png) |
| **2.g** | 静止车辆占用部分车道 | `StaticCarCrossScene` | ![2g](10.png) |

### 3. 周边车辆行驶状态识别及响应

| 场景编码 | 测试场景（Excel） | 场景类名 | 示意图 |
|---------|-----------------|---------|--------|
| **3.a** | 行人通过人行横道线(1) | `PedestrianCrossScene` | ![3a](11.png) |
| **3.b I** | 行人沿道路行走Ⅰ（1） | `PedestrianCrossScene` | ![3b1](12.png) |
| **3.b II** | 行人沿道路行走Ⅱ（1） | `PedestrianCrossScene` | ![3b2](13.png) |
| **3.c** | 自行车同车道骑行（1） | `BicycleCrossScene` | ![3c](14.png) |
| **3.d** | 行人目标感知受阻（1） | `OccludedPedestrianScene` | ![3d](15.png) |
| **4.a** | 前方车辆切入 | `CarCutInScene` | ![4a](16.png) |
| **4.b** | 前方车辆切出 | `CarCutOutScene` | ![4b](17.png) |
| **4.c** | 对向车辆借道行驶(1) | `CarOncomingPassScene` | ![4c](18.png) |
| **4.d** | 目标车辆停-走 | `CarStopandGoScene` | ![4d](19.png) |

### 5. 自动紧急避险

| 场景编码 | 测试场景（Excel） | 场景类名 | 示意图 |
|---------|-----------------|---------|--------|
| **5.a** | 行人横穿道路 | `PedestrianCrossScene` | ![5a](20.png) |
| **5.b** | 自行车横穿道路 | `BicycleCrossScene` | ![5b](21.png) |
| **5.c** | 目标车辆切出后存在静止车辆 | `CarCutOutandStaticScene` | ![5c](22.png) |
| **5.d** | 前方车辆紧急制动 | `CarGoandStopScene` | ![5d](23.png) |
| **5.e** | 紧急转弯危险情况 | `CarCrossScene` | ![5e](24.png) |
| **5.f** | 静止行人目标误触发 | `StaticPedestrianCrossScene` | ![5f](25.png) |
| **5.g** | 移动行人目标误触发 | `PedestrianCrossScene` | ![5g](26.png) |

### 6. 停车

| 场景编码 | 测试场景（Excel） | 场景类名 | 示意图 |
|---------|-----------------|---------|--------|
| **6.a** | 停车点 | `EgoRouteFollowScene` | ![6a](27.png) |
| **6.b** | 港湾式站台 | `EgoRouteFollowScene` | ![6b](27.png) |
| **6.c** | 普通站台 | `EgoRouteFollowScene` | ![6c](27.png) |

## 绘制及启动详细参数
[绘图及运行详细文档](waypoints操作说明.md)

## 断点重续功能
### 新增参数

`--resume` ：启用断点恢复（默认关闭）
### 运行机制
`保存`：每完成一个场景，自动将已完成场景集合 + 测试记录写入{scenario}_checkpoint.pkl，同时增量更新 {scenario}_result.pkl。

`恢复`：再次运行加上 --resume 参数，程序会：
  1. 检查 {scenario}_checkpoint.pkl 是否存在
  2. 如果存在，读取已完成的场景名称集合和已有记录
  3. 跳过已完成的场景，从下一个未完成的场景继续运行
  4. 新完成的场景追加到记录中

###  使用方式
```bash
  # 首次运行（正常执行）
  python tools/run.py --input_dir ./save_scenarios/ --town TOWN10HD_Opt --scenario 3a

  # 中断后恢复（跳过已完成的场景）
  python tools/run.py --input_dir ./save_scenarios/ --town TOWN10HD_Opt --scenario 3a --resume

  断点文件保存在 videos/{scenario}/{scenario}_checkpoint.pkl，与视频和结果文件在同一目录。
```
