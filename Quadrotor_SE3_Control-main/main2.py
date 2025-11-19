# 20250220 Wakkk
# Quadrotor SE3 Control Demo
import mujoco 
import mujoco.viewer as viewer 
import numpy as np
from se3_controller import *
from motor_mixer import *

gravity = 9.8066        # 重力加速度 单位m/s^2
mass = 0.033            # 飞行器质量 单位kg
Ct = 3.25e-4            # 电机推力系数 (N/krpm^2)
Cd = 7.9379e-6          # 电机反扭系数 (Nm/krpm^2)

arm_length = 0.065/2.0  # 电机力臂长度 单位m
max_thrust = 0.1573     # 单个电机最大推力 单位N (电机最大转速22krpm)
max_torque = 3.842e-03  # 单个电机最大扭矩 单位Nm (电机最大转速22krpm)

# 仿真周期 1000Hz 1ms 0.001s
dt = 0.001

# 根据电机转速计算电机推力
def calc_motor_force(krpm):
    global Ct
    return Ct * krpm**2

# 根据推力计算电机转速
def calc_motor_speed_by_force(force):
    global max_thrust
    if force > max_thrust:
        force = max_thrust
    elif force < 0:
        force = 0
    return np.sqrt(force / Ct)

# 根据扭矩计算电机转速 注意返回数值为转速绝对值 根据实际情况决定转速是增加还是减少
def calc_motor_speed_by_torque(torque):
    global max_torque
    if torque > max_torque:  # 扭矩绝对值限制
        torque = max_torque
    return np.sqrt(torque / Cd)

# 根据电机转速计算电机转速
def calc_motor_speed(force):
    if force > 0:
        return calc_motor_speed_by_force(force)

# 根据电机转速计算电机扭矩
def calc_motor_torque(krpm):
    global Cd
    return Cd * krpm**2

# 根据电机转速计算电机归一化输入
def calc_motor_input(krpm):
    if krpm > 22:
        krpm = 22
    elif krpm < 0:
        krpm = 0
    _force = calc_motor_force(krpm)
    _input = _force / max_thrust
    if _input > 1:
        _input = 1
    elif _input < 0:
        _input = 0
    return _input

# 加载模型回调函数
def load_callback(m=None, d=None):
    mujoco.set_mjcb_control(None)
    m = mujoco.MjModel.from_xml_path('./crazyfile/scene.xml')
    d = mujoco.MjData(m)
    if m is not None:
        mujoco.set_mjcb_control(lambda m, d: control_callback(m, d))  # 设置控制回调函数
    return m, d

# 简易平面八字形轨迹生成 (Figure-Eight / Lissajous 1:2)
def simple_trajectory(time):
    wait_time = 1.5     # 起飞到开始点等待时间
    height = 0.3        # 轨迹高度
    radius = 0.5        # 轨迹大小 (半长轴)
    # 频率参数
    freq_x = 0.15       # X轴频率 (例如 0.15 Hz)
    freq_y = 0.3        # Y轴频率 (X轴频率的两倍，确保是八字形)
    
    T = time - wait_time
    
    # 构建机头朝向 (保持指向切线方向)
    if T < 0:
        start_pos = np.array([0, 0, height])
        start_heading = np.array([1.0, 0.0, 0.0]) # 指向X轴正向
        return start_pos, start_heading
    else:
        # 八字形轨迹 (Lissajous Curve 1:2)
        _x = radius * np.sin(2 * np.pi * freq_x * T)
        _y = radius * np.sin(2 * np.pi * freq_y * T) # 频率是x的两倍
        _z = height
        
        # 计算切线方向作为目标机头朝向 (通过数值微分近似)
        dT = 0.001 # 仿真步长
        _x_next = radius * np.sin(2 * np.pi * freq_x * (T + dT))
        _y_next = radius * np.sin(2 * np.pi * freq_y * (T + dT))
        
        _vx = (_x_next - _x) / dT
        _vy = (_y_next - _y) / dT
        
        # 目标朝向是速度向量的归一化
        speed_vector = np.array([_vx, _vy, 0])
        if np.linalg.norm(speed_vector) < 1e-6:
            _heading = np.array([1.0, 0.0, 0.0]) # 速度为零时指向前方
        else:
            _heading = speed_vector / np.linalg.norm(speed_vector)
            
        return np.array([_x, _y, _z]), _heading  # Trajectory Point And Heading

# 初始化SE3控制器
ctrl = SE3Controller()
# 设置参数
ctrl.kx = 0.6
ctrl.kv = 0.4
ctrl.kR = 6.0
ctrl.kw = 1.0
# 初始化电机动力分配器
mixer = Mixer()
torque_scale = 0.001 

log_count = 0

# =========================================================================
# 🛠️ 轨迹可视化相关全局变量和回调函数
# 存储轨迹点的全局列表
trajectory_geoms = [] 
VIS_INTERVAL = 100 # 每100个仿真步（0.1秒）记录一个点
vis_count = 0

# 修复：使用 mjv_addGeoms 风格的回调函数
def add_geom_callback(m, d, scn):
    """
    注册到 user_scn.add_geoms 的回调函数，用于添加自定义几何体。
    在每次渲染前调用。
    """
    global trajectory_geoms
    
    for geom_data in trajectory_geoms:
        # 检查是否有空间添加新的几何体
        if scn.ngeom >= scn.maxgeom:
            break
            
        # 使用 mjv_initGeom 初始化新的几何体结构
        mujoco.mjv_initGeom(
            scn.geoms[scn.ngeom],
            geom_data['type'],
            geom_data['size'],
            geom_data['pos'],
            np.eye(3),  # 旋转矩阵 (单位矩阵)
            geom_data['rgba'],
            None, None, None, None, 0
        )
        scn.ngeom += 1
# =========================================================================

def control_callback(m, d):
    global log_count, vis_count, gravity, mass, dt, trajectory_geoms

    _pos = d.qpos
    _vel = d.qvel
    _acc = d.qacc

    _sensor_data = d.sensordata
    gyro_x = _sensor_data[0]
    gyro_y = _sensor_data[1]
    gyro_z = _sensor_data[2]
    acc_x = _sensor_data[3]
    acc_y = _sensor_data[4]
    acc_z = _sensor_data[5]
    quat_w = _sensor_data[6]
    quat_x = _sensor_data[7]
    quat_y = _sensor_data[8]
    quat_z = _sensor_data[9]
    quat = np.array([quat_x, quat_y, quat_z, quat_w])  # x y z w
    omega = np.array([gyro_x, gyro_y, gyro_z])  # 角速度

    # 构建目标状态
    goal_pos, goal_heading = simple_trajectory(d.time)        # 目标位置 (八字形轨迹)

    goal_vel = np.array([0, 0, 0])              # 目标速度
    goal_quat = np.array([0.0,0.0,0.0,1.0])     # 目标四元数(无用)
    goal_omega = np.array([0, 0, 0])            # 目标角速度
    goal_state = State(goal_pos, goal_vel, goal_quat, goal_omega)
    # 构建当前状态
    curr_state = State(_pos, _vel, quat, omega)

    # =====================================================================
    # 🛠️ 轨迹可视化逻辑 (在控制循环中记录点)
    vis_count += 1
    if vis_count >= VIS_INTERVAL:
        vis_count = 0
        
        # 记录目标位置作为一个小的球体 
        geom_data = {
            'type': mujoco.mjtGeom.mjGEOM_SPHERE, # 球体
            'pos': goal_pos.copy(),               # 目标位置 
            'size': np.array([0.005, 0.0, 0.0]),  # 半径 0.005m
            'rgba': np.array([0.0, 0.0, 1.0, 1.0]) # 蓝色
        }
        trajectory_geoms.append(geom_data)
        
        # 保持轨迹点数量在一个合理范围内，例如只保留最新的2000个点
        if len(trajectory_geoms) > 200:
            trajectory_geoms.pop(0)
    # =====================================================================

    # 更新控制器
    forward = goal_heading
    control_command = ctrl.control_update(curr_state, goal_state, dt, forward)
    ctrl_thrust = control_command.thrust    # 总推力控制量(mg为单位)
    ctrl_torque = control_command.angular   # 三轴扭矩控制量

    # Mixer
    mixer_thrust = ctrl_thrust * gravity * mass     # 机体总推力(N)
    mixer_torque = ctrl_torque * torque_scale       # 机体扭矩(Nm)
    # 输出到电机
    motor_speed = mixer.calculate(mixer_thrust, mixer_torque[0], mixer_torque[1], mixer_torque[2]) # 动力分配
    d.actuator('motor1').ctrl[0] = calc_motor_input(motor_speed[0])
    d.actuator('motor2').ctrl[0] = calc_motor_input(motor_speed[1])
    d.actuator('motor3').ctrl[0] = calc_motor_input(motor_speed[2])
    d.actuator('motor4').ctrl[0] = calc_motor_input(motor_speed[3])

    log_count += 1
    if log_count >= 500:
        log_count = 0
        # ... (Log outputs)


if __name__ == '__main__':
    # 1. 启动 Viewer 并获取 Viewer 实例
    viewer = viewer.launch(loader=load_callback)
    
    # 2. 注册自定义几何体添加回调 (修复了变量覆盖问题)
    viewer.user_scn.add_geoms = add_geom_callback