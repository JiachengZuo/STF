# CARLA Map Editor \(0916\) 核心操作说明

# 一、核心操作（绘制图像\+摆放元素）

## 1\. 绘制行驶路线

- 按住 `Alt` \+ 鼠标右键：在地图上点击，添加路线点（绿色圆圈，标注\&\#34;P1、P2\.\.\.\&\#34;）；

- 路线点会自动吸附到CARLA地图的车道上，连续点击即可绘制完整行驶路线；

- 按 `C` 键：清空已绘制的全部路线点，重新绘制。

## 2\. 摆放Ego车辆（必选）

- 按住 `Ctrl` \+ 鼠标左键：在地图上点击，设置Ego车辆起始点（蓝色圆圈，标注\&\#34;EGO\&\#34;）；

- Ego车辆是场景核心，必须设置，否则无法生成场景文件；

- 选中Ego点（点击蓝色圆圈），按 `Delete` 键：删除当前Ego起始点，重新摆放。

## 3\. 摆放Trigger触发点（必选）

- 按住 `Shift` \+ 鼠标左键：在地图上点击，设置触发点（红色圆圈，标注\&\#34;TRIGGER\&\#34;）；

- 触发点用于触发场景逻辑，需与Ego车辆路线合理搭配；

- 选中Trigger点（点击红色圆圈），按`Delete` 键：删除当前触发点，重新摆放。

## 4\. 摆放Agent智能体（可选）

- 鼠标右键（不按任何快捷键）：在地图上点击，添加默认智能体（行人，蓝色圆圈）；

- 选中智能体（点击蓝色圆圈），按对应数字键切换类型：
        

    - 5：行人（person）

    - 6：自行车（bike）

    - 7：汽车（car）

    - 8：障碍物（obstacle）

- 选中智能体，按对应数字键切换朝向：

    - 1：沿车道正向

    - 2：沿车道反向（180°）

    - 3：车道左侧（\-90°）

    - 4：车道右侧（\+90°）

- 选中智能体，按 `Delete` 键：删除当前选中的智能体。

# 二、运行指令（核心）

## 1\. 前提准备

先启动CARLA服务器，步骤如下：

1. 打开CARLA安装目录，找到`CarlaUE4\.exe`（Windows）或`CarlaUE4\.sh`（Linux）；

2. 双击启动，默认端口为2000（无需修改，与编辑器默认端口一致）；

3. 等待服务器加载完成（界面显示CARLA地图即启动成功）。

## 2\. 运行编辑器

在VS Code终端或系统终端中，切换到程序所在目录，执行以下命令（默认参数）：

```bash
python 文件名.py  # 替换为你的编辑器程序文件名（如carla_map_editor.py）
```

## 3\. 生成场景文件

完成路线绘制、Ego点、Trigger点及智能体摆放后，按 `S` 键：

- 程序会自动生成64个包含不同天气的场景JSON文件；

- 生成完成后，终端会提示进度，同时自动清空当前编辑内容，可继续绘制下一组场景。

## 4\. 自定义参数运行（可选）

若需自定义CARLA服务器地址、地图、场景编号等，执行以下命令：

```bash
python tools/waypoints.py --name roadside_1 --scenario 1 --save_dir ./save_scenarios
```

参数说明（核心常用）：

- \-\-host：CARLA服务器IP（本地默认127\.0\.0\.1）

- \-\-port：CARLA服务器端口（默认2000）

- \-\-name：CARLA地图名称（如TOWN01、TOWN10HD\_Opt）

- \-\-scenario：场景编号（用于生成JSON文件名）

- \-\-save\_dir：场景JSON文件保存目录（默认output文件夹）

> （注：文档部分内容可能由 AI 生成）

## 5\. run.py参考运行指令

### 基础运行

```bash
python run.py --input_dir ./save_scenarios/ --town Town10HD_Opt --scenario 3a
```

### 带碰撞增强运行（推荐）

默认使用 `collision_config.yaml` 提升碰撞率（默认启用，无需额外参数）：

```bash
python run.py --input_dir ./save_scenarios/ --town Town10HD_Opt --scenario 3a
```

### 禁用碰撞增强

```bash
# 编辑 collision_config.yaml，将 global.enabled 设为 false
# 或指定禁用配置:
python run.py --input_dir ./save_scenarios/ --town Town10HD_Opt --scenario 3a --collision_config ./disable_collision.yaml
```

### 自定义碰撞配置

```bash
python run.py --input_dir ./save_scenarios/ --town Town10HD_Opt --scenario 3a --collision_config /path/to/custom.yaml
```

### 完整参数说明

| 参数 | 说明 | 默认值 |
|---|---|---|
| `--input_dir` | 场景JSON文件目录 | 必填 |
| `--town` | CARLA地图名称 | 必填 |
| `--scenario` | 场景类型 (3a/2c/2d/2e/4a/4b/4c/4d/5a~5g/6a~6c) | 3a |
| `--model` | 控制模型 (behavior / tcp) | behavior |
| `--model_path` | TCP模型权重路径 | ./tcp/best_model.ckpt |
| `--video_dir` | 视频输出目录 | videos |
| `--route_id` | 路线ID | route_01 |
| `--collision_config` | 碰撞增强YAML配置路径 | collision_config.yaml |
| `--npc_total` | 环境NPC总数（汽车+自行车+行人） | 0（不生成） |
| `--npc_car_ratio` | NPC中汽车比例 | 0.50 |
| `--npc_cyclist_ratio` | NPC中自行车/摩托车比例 | 0.30（剩余为行人） |

### 环境NPC生成说明

使用 **Traffic Manager** 和 **Walker AI** 在场景周围随机生成自动驾驶的交通流量：

- **车辆（`--npc_car_ratio`）**：从地图出生点随机生成，使用Traffic Manager自动驾驶，随机颜色和速度偏移
- **自行车/摩托车（`--npc_cyclist_ratio`）**：从地图剩余出生点生成，同样使用Traffic Manager控制
- **行人（剩余比例）**：从导航点随机生成，使用Walker AI控制器自动漫游

示例 — 生成20个NPC，包含汽车、自行车和行人：

```bash
python run.py --input_dir ./save_scenarios/ --town TOWN10HD_Opt --scenario 2b --npc_total 20 --npc_car_ratio 0.50 --npc_cyclist_ratio 0.30
```

示例 — 只生成行人（适合行人场景测试）：

```bash
python run.py --input_dir ./save_scenarios/ --town TOWN10HD_Opt --scenario 3a --npc_total 10 --npc_car_ratio 0.0 --npc_cyclist_ratio 0.0
```

示例 — 生成大量车辆交通流：

```bash
python run.py --input_dir ./save_scenarios/ --town TOWN10HD_Opt --scenario 4a --npc_total 30 --npc_car_ratio 0.90 --npc_cyclist_ratio 0.0
```

> **注意**：`npc_total=0`（默认）时不生成任何环境NPC，仅在场景本身定义的agent参与测试。

### 碰撞增强机制

通过 `collision_enhancer.py` + `collision_config.yaml` 实现，提升场景碰撞率：

1. **缩短触发距离** — NPC离ego更近才触发动作（`trigger_distance_scale: 0.7`）
2. **提高NPC速度** — 行人/车辆移动更快（`speed_boost: 1.3`）
3. **延长场景超时** — 给碰撞更多发生时间（`timeout_scale: 1.2`）
4. **GA遗传算法** — 优化NPC初始位置/速度/触发距离，寻找最危险配置

配置示例（`collision_config.yaml`）：

```yaml
global:
  enabled: true          # 总开关，设为 false 禁用所有增强
  ga:
    enabled: true        # 遗传算法优化 NPC 初始状态
collision_profile:
  trigger_distance_scale: 0.7   # < 1.0 = 触发更近
  deceleration_scale: 0.6       # < 1.0 = 刹车更晚
  speed_boost: 1.3              # > 1.0 = NPC 更快
  timeout_scale: 1.2            # > 1.0 = 超时更长
```
