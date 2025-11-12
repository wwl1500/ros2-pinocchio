# Pinocchio ROS2 综合指南

本仓库展示如何在仅安装了 ROS 2 的 Ubuntu 系统上，额外部署 Pinocchio 并在 ROS 2 包中调用其动力学算法。

> 约定环境：Ubuntu 24.04 + ROS 2 Jazzy。

## 目录
1. [适用场景与前置检查](#适用场景与前置检查)
2. [一次性准备步骤](#一次性准备步骤)
   - 更新系统与安装依赖
   - 配置 robotpkg 软件源
   - 安装 Pinocchio
   - 设置环境变量
3. [ROS 2 工作空间配置](#ros-2-工作空间配置)
4. [编译与运行示例](#编译与运行示例)
5. [动力学方程与示例代码](#动力学方程与示例代码)
6. [日常使用速查](#日常使用速查)
7. [常见问题与排查](#常见问题与排查)
8. [更多资源](#更多资源)

---
快速自检命令：
```bash
ros2 --version
lsb_release -a
```

---

## 一次性准备步骤

### 1. 更新系统并安装基础依赖
```bash
sudo apt update
sudo apt upgrade -y  # 可选，保持系统最新

sudo apt install -y \
  build-essential \
  cmake \
  git \
  pkg-config \
  curl \
  software-properties-common \
  libeigen3-dev \
  liburdfdom-dev \
  libassimp-dev \
  libboost-all-dev
```

### 2. 配置 robotpkg 软件源
Pinocchio 推荐通过 `robotpkg` 仓库安装。
```bash
sudo sh -c 'echo "deb [arch=amd64] http://robotpkg.openrobots.org/wip/packages/debian/pub $(lsb_release -cs) robotpkg" > /etc/apt/sources.list.d/robotpkg.list'
curl -s http://robotpkg.openrobots.org/packages/debian/robotpkg.key | sudo apt-key add -
sudo apt update
```

### 3. 安装 Pinocchio
```bash
sudo apt install -y robotpkg-pinocchio
# 如需 Python 绑定或官方示例，可额外安装：
# sudo apt install -y robotpkg-pinocchio-python robotpkg-pinocchio-example
```
验证版本：
```bash
pkg-config --modversion pinocchio
```

### 4. 设置环境变量
Pinocchio 默认安装在 `/opt/openrobots/`。请将以下内容追加到 `~/.bashrc`（或你的 Shell 配置文件）中：
```bash
cat <<'EOF' >> ~/.bashrc
# Pinocchio 环境变量
export PATH="/opt/openrobots/bin:$PATH"
export PKG_CONFIG_PATH="/opt/openrobots/lib/pkgconfig:$PKG_CONFIG_PATH"
export LD_LIBRARY_PATH="/opt/openrobots/lib:$LD_LIBRARY_PATH"
export PYTHONPATH="/opt/openrobots/lib/python3/dist-packages:$PYTHONPATH"
EOF
source ~/.bashrc
```
验证：
```bash
printenv PKG_CONFIG_PATH | grep openrobots
```
若能看到 `/opt/openrobots/lib/pkgconfig`，说明配置成功。

---

## ROS 2 工作空间配置

1. 若尚未创建工作空间：
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws
   ```
2. 克隆或拷贝本示例包至 `src/`，例如：
   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/yourname/pinocchio_ros2_example.git
   ```
3. 回到工作空间根目录并加载 ROS 2 环境：
   ```bash
   cd ~/ros2_ws
   source /opt/ros/jazzy/setup.bash
   ```
4. 可选：将以上两条 `source` 命令写入 `~/.bashrc`，方便今后自动加载环境。

---

## 编译与运行示例

```bash
cd ~/ros2_ws
colcon build --packages-select pinocchio_ros2_example
source install/setup.bash
```

运行节点与话题：
```bash
# C++ 节点
ros2 run pinocchio_ros2_example pinocchio_node

# C++ 流体动力节点（实时外力发布 tau_ext_cpp）
ros2 run pinocchio_ros2_example fluid_dynamics_cpp_node --ros-args \
  -p urdf_path:=/home/wwl/ros2-pinocchio/install/pinocchio_ros2_example/share/pinocchio_ros2_example/models/two_link.urdf

# Python 示例节点
ros2 run pinocchio_ros2_example pinocchio_python_node.py

# Python 流体动力外力节点（参考 111.tex）
ros2 launch pinocchio_ros2_example fluid_dynamics.launch.py

# C++ 流体动力外力节点（参考 111.tex，性能更高）
ros2 launch pinocchio_ros2_example fluid_dynamics_cpp.launch.py
```

---

## 动力学方程与示例代码

Pinocchio 使用的标准动力学形式：
```
M(q)\ddot{q} + C(q, \dot{q})\dot{q} + g(q) = \tau + \tau_{ext}
```
- `M(q)`: 惯性矩阵
- `C(q, \dot{q})\dot{q}`: 科氏/离心项
- `g(q)`: 重力项
- `τ`: 控制力矩
- `τ_ext`: 外部力矩（如流体阻力）

### C++ 关键片段（详见 `src/pinocchio_node.cpp`）
```cpp
// 计算惯性矩阵
pinocchio::crba(model, data, q);
Eigen::MatrixXd M = data.M;

// 计算科氏项与重力项
pinocchio::computeAllTerms(model, data, q, v);
Eigen::VectorXd Cv = data.C * v;
Eigen::VectorXd g = data.g;

// 前向动力学：解加速度
a = M.ldlt().solve(tau + tau_ext - (Cv + g));

// 逆动力学：给定 q, v, a 求力矩
Eigen::VectorXd tau_required = pinocchio::rnea(model, data, q, v, a);
```

### Python 关键片段（详见 `scripts/pinocchio_python_node.py`）
```python
import pinocchio
import numpy as np

pinocchio.crba(model, data, q)
M = data.M
pinocchio.computeAllTerms(model, data, q, v)
Cv = data.C @ v
g = data.g

qdd = np.linalg.solve(M, tau + tau_ext - (Cv + g))

tau_required = pinocchio.rnea(model, data, q, v, a)
```

### 自定义外部力矩示例（C++）
```cpp
Eigen::VectorXd compute_fluid_forces(const Eigen::VectorXd &q, const Eigen::VectorXd &v)
{
  Eigen::MatrixXd D = Eigen::MatrixXd::Identity(model.nv, model.nv) * 0.1; // 简单阻尼
  return -D * v;  // 流体阻力
}
```
将其加入外部力矩 `tau_ext`，即可模拟水下或空气动力学影响。

---

## 基于 IEEE 6907522 的流体动力节点

文件 `scripts/fluid_dynamics_node.py`（Python 版本）、`src/fluid_dynamics_cpp/fluid_dynamics_cpp_node.cpp`（C++ 版本）以及对应的 launch 文件 `launch/fluid_dynamics.launch.py` 和 `launch/fluid_dynamics_cpp.launch.py` 实现了 IEEE 6907522 文献 ["Modeling of underwater snake robots"](https://ieeexplore.ieee.org/document/6907522) 中描述的流体动力学模型：

**参考文献：**
- IEEE 6907522: "Modeling of underwater snake robots"
- 理论推导参考：`/home/wwl/My_Note/水下蛇形机器人动力学推导详述/水下蛇形机器人动力学推导详述.tex`

**实现的流体动力学模型：**
- **附加质量（Added Mass）**：对应文献中的 μ_n, μ_t, λ_1
- **线性阻尼（Linear Damping）**：对应文献中的 c_t, c_n
- **非线性阻尼（Nonlinear Damping）**：对应文献中的二次阻力项
- **未知洋流扰动**：以参数形式进入，可置零

**核心做法：** 将上述模型在各受力帧上计算出的空间力矩注入到 Pinocchio 的 `tau_ext` 中，使用 `pinocchio.rnea(model, data, q, dq, ddq, fext)` 得到流体外力贡献。实现公式：`M(q)q̈ + C(q, q̇)q̇ + g(q) = τ + τ_ext`

### 快速运行

#### Python 版本
```bash
# 1. 编译并加载环境
colcon build --packages-select pinocchio_ros2_example
source install/setup.bash

# 2. 启动 Python 流体动力节点（默认加载示例 two_link.urdf）
ros2 launch pinocchio_ros2_example fluid_dynamics.launch.py
```

#### C++ 版本
```bash
# 1. 编译并加载环境（确保已编译 C++ 节点）
colcon build --packages-select pinocchio_ros2_example
source install/setup.bash

# 2. 启动 C++ 流体动力节点（默认加载示例 two_link.urdf）
ros2 launch pinocchio_ros2_example fluid_dynamics_cpp.launch.py
```

两个版本功能相同，C++ 版本性能更高，适合实时控制场景。

### 主要参数（可在 launch 文件中覆盖）
- `urdf_path`：机器人 URDF 路径，默认 `share/pinocchio_ros2_example/models/two_link.urdf`
- `fluid_frames`：受流体作用的帧列表（默认 `['link_1', 'link_2']`）
- `free_flyer`：是否启用自由基座
- `v_current_world`：世界系洋流速度矢量
- 帧级参数（通过 `ros2 param set` 或 YAML 传入）：`<frame>.m_u / m_v / m_r`、`<frame>.d_lu / d_lv / d_lr`、`<frame>.d_nu / d_nv / d_nr`

节点会在话题 `tau_ext`（Python 版本）或 `tau_ext_cpp`（C++ 版本）发布广义流体力矩，可与控制器输出叠加后发送到执行器或用于仿真。

### 带外力公式验证（在线/离线）

节点内部和独立验证脚本均支持对公式进行数值验证：

- 验证公式：`M(q) q̈ + C(q, q̇) q̇ + g(q) = τ_with − τ_ext`
- 期望残差：约 `1e-8`（双精度数值误差量级内）

#### 在线（节点内置一次性验证）
```bash
source /home/wwl/ros2-pinocchio/install/setup.bash
ros2 launch pinocchio_ros2_example fluid_dynamics.launch.py start_sine_pub:=true
# 首次收到 /joint_states 后，会在终端打印：
# 验证 公式: ||M*qdd + C*v + g - (tau - tau_ext)|| = 0.000e+00
```

#### 离线（独立验证脚本，持续验证）
1) 启动产生外力的节点（Python 版示例）  
```bash
source /home/wwl/ros2-pinocchio/install/setup.bash
ros2 launch pinocchio_ros2_example fluid_dynamics.launch.py start_sine_pub:=true
```
2) 另开终端运行验证器（订阅 `/tau_ext`，也可改为 `/tau_ext_cpp`）  
```bash
source /home/wwl/ros2-pinocchio/install/setup.bash
ros2 run pinocchio_ros2_example fluid_dynamics_verifier.py \
  --ros-args \
  -p urdf_path:=/home/wwl/ros2-pinocchio/install/pinocchio_ros2_example/share/pinocchio_ros2_example/models/two_link.urdf \
  -p fluid_frames:="['link_1','link_2']" \
  -p rho:=1000.0 \
  -p tau_topic:=/tau_ext
```
3) 可选：过滤查看验证日志  
```bash
source /home/wwl/ros2-pinocchio/install/setup.bash
ros2 topic echo /rosout | grep --line-buffered fluid_dynamics_verifier
```
输出示例：
```
核对结果: ||tau_ext_calc|| = 5.0e+01, ||residual|| = 0.0e+00, 与话题差值范数 = 1.2e+02
```
- `||residual|| ≈ 0`：公式成立  
- `||tau_ext_calc||`：按当前流体模型重建的外力矩范数  
- `与话题差值范数`：与实际话题外力的差异（用于交叉校核）

---

## 日常使用速查

常用命令（假设环境变量已写入 `~/.bashrc`）：
```bash
# 切换到工作空间
cd ~/ros2_ws

# 更新代码后重新编译
colcon build --packages-select pinocchio_ros2_example
source install/setup.bash

# 运行节点
ros2 run pinocchio_ros2_example pinocchio_node
```

验证 Pinocchio 安装：
```bash
pkg-config --modversion pinocchio
python3 -c "import pinocchio; print(pinocchio.__version__)"
```


## 参考资料
- Pinocchio 官方文档：https://stack-of-tasks.github.io/pinocchio/
- Pinocchio GitHub：https://github.com/stack-of-tasks/pinocchio
- ROS 2 文档：https://docs.ros.org/
- 示例源码：`src/pinocchio_node.cpp`、`scripts/pinocchio_python_node.py`

