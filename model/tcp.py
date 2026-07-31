"""
TCP (Trajectory-guided Control Prediction) 自动驾驶算法的Safebench集成
基于TCP论文的端到端自动驾驶算法实现

TCP需要的输入:
- RGB图像 (900x256, FOV=100)
- 速度 (m/s)
- 目标点 (相对于车辆的局部坐标)
- 导航命令 (通过RoutePlanner获取)

TCP的输出:
- throttle, steer, brake 控制信号
"""

import os
import sys
import math
import numpy as np
from collections import deque, OrderedDict

import torch
from torchvision import transforms as T

from agent.base_policy import BasePolicy

# 添加TCP目录到sys.path以导入TCP模型
TCP_ROOT = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), 'TCP')
if TCP_ROOT not in sys.path:
    sys.path.insert(0, TCP_ROOT)

from TCP.TCP.model import TCP
from TCP.TCP.config import GlobalConfig


class TCPRoutePlanner:
    """
    为TCP设计的路线规划器，基于safebench的route信息计算目标点
    """
    def __init__(self, min_distance=4.0, max_distance=50.0):
        self.route = deque()
        self.min_distance = min_distance
        self.max_distance = max_distance
        # CARLA 9.10+ 的坐标系参数
        self.mean = np.array([0.0, 0.0])
        self.scale = np.array([111324.60662786, 111319.490945])
        
    def set_route(self, waypoints_list):
        """
        设置路线
        Args:
            waypoints_list: 包含carla.Waypoint对象的列表
        """
        self.route.clear()
        for waypoint in waypoints_list:
            pos = np.array([waypoint.transform.location.x, waypoint.transform.location.y])
            # 使用waypoint的road_option或默认LANEFOLLOW
            cmd = getattr(waypoint, 'road_option', 4)  # 4 = LANEFOLLOW
            self.route.append((pos, cmd))
    
    def run_step(self, vehicle_location):
        """
        根据当前车辆位置计算下一个目标点
        Args:
            vehicle_location: 当前车辆位置 (x, y)
        Returns:
            next_wp: 下一个路点位置
            next_cmd: 导航命令
        """
        if len(self.route) <= 1:
            if len(self.route) == 1:
                return self.route[0]
            return (np.array([0.0, 0.0]), 4)
        
        pos = np.array([vehicle_location.x, vehicle_location.y])
        
        # 移除已通过的路点
        to_pop = 0
        for i in range(len(self.route)):
            distance = np.linalg.norm(self.route[i][0] - pos)
            if distance <= self.min_distance:
                to_pop = i + 1
        
        for _ in range(min(to_pop, len(self.route) - 2)):
            self.route.popleft()
        
        # 返回下一个目标点
        if len(self.route) > 1:
            return self.route[1]
        return self.route[0]


class TCPAgent(BasePolicy):
    """
    TCP (Trajectory-guided Control Prediction) 自动驾驶智能体
    
    实现了BasePolicy接口，可以在safebench框架中使用
    """
    name = 'tcp'
    type = 'unlearnable'  # 预训练模型，不需要在线学习
    
    def __init__(self, model_path, planner):
        super().__init__(model_path, planner)
        self.model_path = model_path
        self.mode = 'eval'
        self.continue_episode = 0
        
        # TCP配置
        self.config = GlobalConfig()
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        
        # 初始化模型
        self.net = None
        
        # 图像预处理
        self._im_transform = T.Compose([
            T.ToTensor(), 
            T.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
        ])
        self.planner_list = []
        self.planner_list.append(planner)
        
        # 用于控制平滑的状态
        self.last_steers = [deque(maxlen=20) for _ in range(1)]
        self.status_list = [0] * 1  # 0: use traj, 1: use ctrl
        self.alpha = 0.3  # 控制融合权重
        self.step_count = [0] * 1
        
    def set_ego_and_route(self, ego_vehicles, info, static_obs=None):
        """
        设置ego车辆和路线信息
        
        Args:
            ego_vehicles: ego车辆列表
            info: 包含route_waypoints等信息的字典列表
            static_obs: 静态观测信息
        """
        self.ego_vehicles = ego_vehicles
        self.planner_list = []
        self.last_steers = [deque(maxlen=20) for _ in range(len(ego_vehicles))]
        self.status_list = [0] * len(ego_vehicles)
        self.step_count = [0] * len(ego_vehicles)
        
        for e_i in range(len(ego_vehicles)):
            planner = TCPRoutePlanner(min_distance=4.0, max_distance=50.0)
            # 从info中获取路线waypoints
            if 'route_waypoints' in info[e_i]:
                planner.set_route(info[e_i]['route_waypoints'])
            self.planner_list.append(planner)
    
    def train(self, replay_buffer):
        """TCP是预训练模型，不需要训练"""
        pass
    
    def set_mode(self, mode):
        """设置模式（train/eval）"""
        self.mode = mode
        if self.net is not None:
            if mode == 'eval':
                self.net.eval()
            else:
                self.net.train()
    
    def load_model(self):
        """加载TCP预训练模型"""
        self.net = TCP(self.config)
        
        if self.model_path and os.path.exists(self.model_path):

            
            ckpt = torch.load(self.model_path, map_location=self.device, weights_only=False)
            
            # 处理checkpoint格式
            if 'state_dict' in ckpt:
                state_dict = ckpt['state_dict']
            else:
                state_dict = ckpt
            
            # 移除 "model." 前缀（如果有）
            new_state_dict = OrderedDict()
            for key, value in state_dict.items():
                new_key = key.replace("model.", "")
                new_state_dict[new_key] = value
            
            self.net.load_state_dict(new_state_dict, strict=False)

        else:
            print(f'>> Warning: TCP model path not found: {self.model_path}', 'yellow')
        
        self.net.to(self.device)
        self.net.eval()
    
    def save_model(self, episode=None):
        """TCP是预训练模型，不需要保存"""
        pass
    
    def _get_speed(self, vehicle):
        """获取车辆速度 (m/s)"""
        v = vehicle.get_velocity()
        return math.sqrt(v.x**2 + v.y**2 + v.z**2)
    
    def _get_compass(self, vehicle):
        """获取车辆朝向角度 (弧度)"""
        transform = vehicle.get_transform()
        return math.radians(transform.rotation.yaw)
    
    def _compute_target_point(self, e_i, ego_vehicle):
        """
        计算目标点（相对于车辆的局部坐标）
        
        Args:
            e_i: 场景索引
            ego_vehicle: ego车辆
            
        Returns:
            target_point: 相对于车辆的目标点坐标 [x, y]
        """
        transform = ego_vehicle.get_transform()
        location = transform.location
        
        # 获取下一个路点
        next_wp, next_cmd = self.planner_list[e_i].run_step(location)
        
        # 计算相对位置
        compass = self._get_compass(ego_vehicle)
        theta = compass + np.pi / 2
        R = np.array([
            [np.cos(theta), -np.sin(theta)],
            [np.sin(theta), np.cos(theta)]
        ])
        
        pos = np.array([location.x, location.y])
        local_command_point = np.array([next_wp[0] - pos[0], next_wp[1] - pos[1]])
        local_command_point = R.T.dot(local_command_point)
        
        return local_command_point, next_cmd
    
    def _prepare_tcp_input(self, obs, e_i, ego_vehicle):
        """
        准备TCP模型的输入
        
        Args:
            obs: 观测数据（可能是字典或numpy数组）
            e_i: 场景索引
            ego_vehicle: ego车辆
            
        Returns:
            rgb: 预处理后的RGB图像tensor
            state: 状态向量tensor
            target_point: 目标点tensor
            speed: 速度tensor
            command: 导航命令值
        """
        # 获取速度
        speed = self._get_speed(ego_vehicle)
        speed_tensor = torch.FloatTensor([speed]).view(1, 1).to(self.device)
        speed_normalized = speed_tensor / 12.0  # 归一化
        
        # 计算目标点
        target_point_np, command = self._compute_target_point(e_i, ego_vehicle)
        target_point = torch.FloatTensor(target_point_np).view(1, 2).to(self.device)
        
        # 处理导航命令
        if isinstance(command, int):
            cmd_value = command
        else:
            cmd_value = getattr(command, 'value', 4)
        
        if cmd_value < 0:
            cmd_value = 4
        cmd_value = min(cmd_value - 1, 5)
        cmd_value = max(0, cmd_value)
        
        # one-hot编码
        cmd_one_hot = [0] * 6
        cmd_one_hot[cmd_value] = 1
        cmd_one_hot = torch.tensor(cmd_one_hot).view(1, 6).to(self.device, dtype=torch.float32)
        
        # 组合state向量
        state = torch.cat([speed_normalized, target_point, cmd_one_hot], dim=1)
        
        # 获取RGB图像
        if isinstance(obs, dict) and 'img' in obs:
            rgb_img = obs['img']
        elif isinstance(obs, dict) and 'camera' in obs:
            rgb_img = obs['camera']
        else:
            # 如果obs是numpy数组或其他格式
            rgb_img = obs
        
        # 确保是numpy数组
        if isinstance(rgb_img, np.ndarray):
            # 调整尺寸到TCP期望的输入 (256, 900) -> 先resize到合适尺寸
            from PIL import Image
            img = Image.fromarray(rgb_img.astype(np.uint8))
            # TCP期望的输入尺寸是 256x900，但我们可能收到不同尺寸
            # 这里resize到TCP训练时使用的尺寸
            img = img.resize((900, 256), Image.BILINEAR)
            rgb_img = np.array(img)
        
        # 应用图像变换
        rgb_tensor = self._im_transform(rgb_img).unsqueeze(0).to(self.device, dtype=torch.float32)
        
        return rgb_tensor, state, target_point, speed_tensor, cmd_value
    
    def _update_status(self, e_i, steer):
        """
        更新控制状态（决定使用traj还是ctrl输出）
        
        Args:
            e_i: 场景索引
            steer: 当前转向值
        """
        self.last_steers[e_i].append(abs(steer))
        
        # 检查是否在转弯
        num_large_steers = sum(1 for s in self.last_steers[e_i] if s > 0.10)
        if num_large_steers > 10:
            self.status_list[e_i] = 1  # 使用ctrl
        else:
            self.status_list[e_i] = 0  # 使用traj
    
    @torch.no_grad()
    def get_action(self, obs, ego_vehicles, deterministic=False):
        """
        获取控制动作
        
        Args:
            obs:image numpy
            deterministic: 是否使用确定性策略
            
        Returns:
            actions: 动作数组 [[throttle, steer], ...]
        """
        if self.net is None:
            self.load_model()
        
        actions = []
        

        ego_vehicle = ego_vehicles
        self.step_count[0] += 1

        # 获取当前场景的观测
        if isinstance(obs, list):
            curr_obs = obs
        elif isinstance(obs, np.ndarray) and len(obs.shape) > 1:
            curr_obs = obs
        else:
            curr_obs = obs
        
        # 前几帧返回零动作（等待初始化）
        if self.step_count[0] <= self.config.seq_len:
            actions.append([0.0, 0.0])
            return np.array(actions, dtype=np.float32)
        

        # 准备输入
        # curr_obs is the numpy of image
        # e_i is the number of scenario
        # ego_vehicle
        rgb, state, target_point, speed_tensor, cmd_value = self._prepare_tcp_input(
            curr_obs, 0, ego_vehicle
        )
        
        # TCP前向推理
        pred = self.net(rgb, state, target_point)
        
        # 使用process_action获取ctrl输出
        steer_ctrl, throttle_ctrl, brake_ctrl, _ = self.net.process_action(
            pred, cmd_value, speed_tensor, target_point
        )
        
        # 使用control_pid获取traj输出
        steer_traj, throttle_traj, brake_traj, metadata = self.net.control_pid(
            pred['pred_wp'], speed_tensor, target_point
        )
        
        # 刹车阈值处理
        if brake_traj < 0.05:
            brake_traj = 0.0
        if throttle_traj > brake_traj:
            brake_traj = 0.0
        
        # 根据状态融合两种控制输出
        if self.status_list == 0:
            # 使用traj为主
            steer = float(np.clip(self.alpha * steer_ctrl + (1 - self.alpha) * steer_traj, -1, 1))
            throttle = float(np.clip(self.alpha * throttle_ctrl + (1 - self.alpha) * throttle_traj, 0, 0.75))
            brake = float(np.clip(self.alpha * brake_ctrl + (1 - self.alpha) * brake_traj, 0, 1))
        else:
            # 使用ctrl为主
            steer = float(np.clip(self.alpha * steer_traj + (1 - self.alpha) * steer_ctrl, -1, 1))
            throttle = float(np.clip(self.alpha * throttle_traj + (1 - self.alpha) * throttle_ctrl, 0, 0.75))
            brake = float(np.clip(self.alpha * brake_traj + (1 - self.alpha) * brake_ctrl, 0, 1))
        
        # 刹车时不加油
        if brake > 0.5:
            throttle = 0.0
        
        # 更新控制状态
        self._update_status(0, steer)
        
        # TCP输出的是throttle/steer，但safebench期望的是[throttle, steer]
        # 注意：如果有强刹车需求，可能需要特殊处理
        actions.append([throttle, steer])
            

        # print(metadata['speed']*3.6)
        return np.array(actions, dtype=np.float32)
