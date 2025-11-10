# Pinocchio ROS2 综合指南

本仓库展示如何在仅安装了 ROS 2 的 Ubuntu 系统上，额外部署 Pinocchio 并在 ROS 2 包中调用其动力学算法。本文档整合了原先分散的快速开始、使用指南与动力学说明，建议从头到尾跟随完成。

> 约定环境：Ubuntu 24.04 + ROS 2 Jazzy。若你使用其他发行版或 ROS 版本，请将命令中的 `jazzy` 替换为实际值。

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

## 适用场景与前置检查

- 你已经完成 ROS 2 官方安装，并能在终端执行 `ros2 --version` 获得正确输出。
- 你拥有 `sudo` 权限。
- 若此前未安装 Pinocchio，可按照下文“一次性准备步骤”操作；若已安装，请直接跳到 [ROS 2 工作空间配置](#ros-2-工作空间配置)。

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
# 终端 1：C++ 节点
ros2 run pinocchio_ros2_example pinocchio_node

# 终端 2：Python 节点（可选）
ros2 run pinocchio_ros2_example pinocchio_python_node.py

# 终端 3：话题监听
ros2 topic echo /pinocchio_info
```

预期输出示例：
```
[INFO] [pinocchio_node]: Pinocchio Node 启动成功
[INFO] [pinocchio_node]: Pinocchio 版本: 3.4.0
...
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

---

## 常见问题与排查

| 问题描述 | 解决思路 |
| --- | --- |
| `pkg-config` 找不到 `pinocchio` | 确认已安装 `robotpkg-pinocchio`，并执行 `source ~/.bashrc` 让环境变量生效。 |
| 编译时报找不到 `/usr/include/eigen3` | 安装 `libeigen3-dev`，并确保 CMake 中加入该目录。 |
| 运行时报缺少 `libpinocchio.so` | 确认 `LD_LIBRARY_PATH` 中包含 `/opt/openrobots/lib`。 |
| Python `import pinocchio` 失败 | 安装 `robotpkg-pinocchio-python` 并检查 `PYTHONPATH`。 |
| 使用 `colcon build` 报 ROS 依赖未找到 | 确保在当前终端已执行 `source /opt/ros/jazzy/setup.bash`。 |

调试建议：查看 `build/` 与 `log/latest_build` 中的详细日志，或运行 `colcon build --event-handlers console_direct+` 获取实时输出。

---

## 更多资源
- Pinocchio 官方文档：https://stack-of-tasks.github.io/pinocchio/
- Pinocchio GitHub：https://github.com/stack-of-tasks/pinocchio
- ROS 2 文档：https://docs.ros.org/
- 示例源码：`src/pinocchio_node.cpp`、`scripts/pinocchio_python_node.py`

