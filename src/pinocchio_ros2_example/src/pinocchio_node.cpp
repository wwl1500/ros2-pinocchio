#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/fwd.hpp>
#include <Eigen/Dense>
#include <memory>

/*
 * 运行本示例前，请确保已按 README.md 中的流程完成：
 *   1. 在仅安装 ROS 2 的 Ubuntu 上通过 robotpkg 安装 Pinocchio；
 *   2. 配置 PATH / PKG_CONFIG_PATH / LD_LIBRARY_PATH / PYTHONPATH 环境变量；
 *   3. 在 ROS 2 工作空间中成功构建本包。
 * 若环境未按 README.md 准备，运行时可能出现链接或依赖错误。
 */

class PinocchioNode : public rclcpp::Node
{
public:
  PinocchioNode() : Node("pinocchio_node")
  {
    // 参数：URDF 路径与是否使用自由基座
    std::string urdf_path = this->declare_parameter<std::string>("urdf_path", "");
    bool free_flyer = this->declare_parameter<bool>("free_flyer", false);

    // 初始化模型：优先尝试加载 URDF
    model_ = std::make_shared<pinocchio::Model>();
    if (!urdf_path.empty())
    {
      try {
        if (free_flyer) {
          pinocchio::urdf::buildModel(urdf_path, pinocchio::JointModelFreeFlyer(), *model_);
        } else {
          pinocchio::urdf::buildModel(urdf_path, *model_);
        }
      } catch (const std::exception &e) {
        RCLCPP_WARN(this->get_logger(), "加载URDF失败: %s", e.what());
      }
    }
    data_ = std::make_shared<pinocchio::Data>(*model_);
    
    RCLCPP_INFO(this->get_logger(), "Pinocchio Node 启动成功");
    RCLCPP_INFO(this->get_logger(), "Pinocchio 版本: %s", PINOCCHIO_VERSION);
    RCLCPP_INFO(this->get_logger(), "模型关节数: %d", model_->njoints);
    
    // 创建发布者
    publisher_ = this->create_publisher<std_msgs::msg::String>("pinocchio_info", 10);
    
    // 创建定时器，每秒发布一次信息并演示动力学计算
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&PinocchioNode::timer_callback, this));
  }

private:
  /**
   * 演示如何使用 Pinocchio 计算动力学方程:
   * M(q)q̈ + C(q, q̇)q̇ + g(q) = τ + τ_ext
   */
  void compute_dynamics_example()
  {
    if (model_->nq == 0) {
      RCLCPP_WARN(this->get_logger(), "模型为空，无法计算动力学。请加载URDF模型。");
      return;
    }
    
    // 1. 定义状态变量
    Eigen::VectorXd q = Eigen::VectorXd::Zero(model_->nq);      // 关节位置
    Eigen::VectorXd v = Eigen::VectorXd::Zero(model_->nv);      // 关节速度
    Eigen::VectorXd a = Eigen::VectorXd::Zero(model_->nv);      // 关节加速度（初始化为0）
    Eigen::VectorXd tau = Eigen::VectorXd::Zero(model_->nv);    // 控制力矩
    Eigen::VectorXd tau_ext = Eigen::VectorXd::Zero(model_->nv); // 外部力矩（如流体作用力）
    
    // 2. 计算惯性矩阵 M(q)
    // 使用 CRBA (Composite Rigid Body Algorithm) 计算惯性矩阵
    pinocchio::crba(*model_, *data_, q);
    Eigen::MatrixXd M = data_->M;  // 惯性矩阵 M(q)
    
    // 3. 计算科氏项/离心项 C(q, q̇)q̇ 和重力项 g(q)
    // 使用 computeAllTerms 一次性计算所有项
    pinocchio::computeAllTerms(*model_, *data_, q, v);
    
    // C(q, q̇)q̇: 科氏项/离心项
    Eigen::VectorXd Cv = data_->C * v;
    
    // g(q): 重力项
    Eigen::VectorXd g = data_->g;
    
    // 4. 添加外部力矩 τ_ext（例如流体作用力）
    // 用户可以在这里定义自己的外部力矩
    // tau_ext = compute_fluid_forces(q, v);  // 示例：计算流体作用力
    
    // 5. 计算完整的动力学方程
    // M(q)q̈ + C(q, q̇)q̇ + g(q) = τ + τ_ext
    // 可以重新排列为：M(q)q̈ = τ + τ_ext - C(q, q̇)q̇ - g(q)
    Eigen::VectorXd rhs = tau + tau_ext - Cv - g;
    
    // 6. 求解加速度 q̈（如果已知力矩）
    // q̈ = M(q)^(-1) * (τ + τ_ext - C(q, q̇)q̇ - g(q))
    Eigen::VectorXd qdd = M.ldlt().solve(rhs);
    
    RCLCPP_INFO(this->get_logger(), "动力学计算完成:");
    RCLCPP_INFO(this->get_logger(), "  - 惯性矩阵 M(q) 维度: %ld x %ld", M.rows(), M.cols());
    RCLCPP_INFO(this->get_logger(), "  - 科氏项 C(q,q̇)q̇ 维度: %ld", Cv.size());
    RCLCPP_INFO(this->get_logger(), "  - 重力项 g(q) 维度: %ld", g.size());
    RCLCPP_INFO(this->get_logger(), "  - 外部力矩 τ_ext 维度: %ld", tau_ext.size());
  }
  
  /**
   * 使用 RNEA (Recursive Newton-Euler Algorithm) 计算逆动力学
   * 给定 q, q̇, q̈，计算所需的力矩 τ
   */
  Eigen::VectorXd compute_inverse_dynamics(
    const Eigen::VectorXd& q,
    const Eigen::VectorXd& v,
    const Eigen::VectorXd& a,
    const Eigen::VectorXd& tau_ext = Eigen::VectorXd::Zero(0))
  {
    // RNEA 计算: τ = M(q)q̈ + C(q, q̇)q̇ + g(q) - τ_ext
    Eigen::VectorXd tau = pinocchio::rnea(*model_, *data_, q, v, a);
    
    // 如果有外部力矩，需要减去
    if (tau_ext.size() == model_->nv) {
      tau -= tau_ext;
    }
    
    return tau;
  }
  
  /**
   * 使用前向动力学计算加速度
   * 给定 q, q̇, τ, τ_ext，计算加速度 q̈
   */
  Eigen::VectorXd compute_forward_dynamics(
    const Eigen::VectorXd& q,
    const Eigen::VectorXd& v,
    const Eigen::VectorXd& tau,
    const Eigen::VectorXd& tau_ext = Eigen::VectorXd::Zero(0))
  {
    // 1. 计算惯性矩阵
    pinocchio::crba(*model_, *data_, q);
    Eigen::MatrixXd M = data_->M;
    
    // 2. 计算非线性项（科氏项 + 重力项）
    // 使用 computeAllTerms 计算所有项，然后组合得到非线性项
    pinocchio::computeAllTerms(*model_, *data_, q, v);
    Eigen::VectorXd nle = data_->C * v + data_->g;  // nle = C(q, q̇)q̇ + g(q)
    
    // 3. 计算总力矩
    Eigen::VectorXd tau_total = tau;
    if (tau_ext.size() == model_->nv) {
      tau_total += tau_ext;
    }
    
    // 4. 求解加速度: M(q)q̈ = τ + τ_ext - (C(q, q̇)q̇ + g(q))
    // 即: q̈ = M(q)^(-1) * (τ + τ_ext - nle)
    Eigen::VectorXd a = M.ldlt().solve(tau_total - nle);
    
    return a;
  }

  void timer_callback()
  {
    // 只执行一次并停止计时器
    auto message = std_msgs::msg::String();
    message.data = "Pinocchio 正在运行 - 版本: " + std::string(PINOCCHIO_VERSION);
    publisher_->publish(message);

    compute_dynamics_example();  // 单次输出
    if (timer_) {
      timer_->cancel();
    }
  }
  
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  std::shared_ptr<pinocchio::Model> model_;
  std::shared_ptr<pinocchio::Data> data_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PinocchioNode>());
  rclcpp::shutdown();
  return 0;
}

