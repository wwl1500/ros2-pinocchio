#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/container/aligned-vector.hpp>

#include <Eigen/Core>

#include <unordered_map>
#include <string>
#include <vector>
#include <memory>
#include <stdexcept>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <functional>

namespace pin_ros2_example
{
/**
 * @brief 帧级流体动力学配置结构体
 * 
 * 参数对应 IEEE 6907522 "Modeling of underwater snake robots" 中的符号：
 * - mu, mv: 附加质量系数 (kg)，对应文献中的 μ_n, μ_t
 * - mr: 附加质量转动惯量 (kg·m²)，对应文献中的 λ_1
 * - dlu, dlv, dlr: 线性阻尼系数，对应文献中的 c_t, c_n
 * - dnu, dnv, dnr: 非线性阻尼系数，对应文献中的二次项
 */
struct FrameConfig
{
  double area{0.01};
  double cd_lin{0.0};
  double cd_quad{0.0};
  double cd_ang_lin{0.1};
  double mu{0.089};   // 沿 u 方向附加质量 (kg)
  double mv{0.089};   // 沿 v 方向附加质量 (kg)
  double mr{0.16};    // 绕 z 轴附加转动惯量 (kg·m²)
  double dlu{0.4};    // u 方向线性阻尼 (N·s/m)
  double dlv{3.0};    // v 方向线性阻尼 (N·s/m)
  double dlr{0.0};    // 旋转线性阻尼 (N·m·s/rad)
  double dnu{12.9};   // u 方向非线性阻尼 (N·s²/m²)
  double dnv{21.3};   // v 方向非线性阻尼 (N·s²/m²)
  double dnr{0.0};    // 旋转非线性阻尼 (N·m·s²/rad²)
  Eigen::Vector3d dist_added = Eigen::Vector3d::Zero();
  Eigen::Vector3d dist_damp = Eigen::Vector3d::Zero();
  Eigen::Vector3d prev_body_vel = Eigen::Vector3d::Zero();
  bool has_prev_body_vel{false};
};

/**
 * @brief 基于 Pinocchio 的流体动力外力注入节点（C++ 实现）
 * 
 * 参考文献：
 *   IEEE 6907522: "Modeling of underwater snake robots"
 *   https://ieeexplore.ieee.org/document/6907522
 * 
 * 实现的流体动力学模型包括：
 * - 附加质量（Added Mass）：对应文献中的 μ_n, μ_t, λ_1
 * - 线性阻尼（Linear Damping）：对应文献中的 c_t, c_n
 * - 非线性阻尼（Nonlinear Damping）：对应文献中的二次阻力项
 * - 未知洋流视为扰动（以参数形式进入，可置零）
 * 
 * 核心实现：
 *   将各受力帧的空间力矩聚合为 joint 外力向量 fext，传入 rnea 获取广义力矩贡献 tau_ext。
 *   实现公式：M(q)q̈ + C(q, q̇)q̇ + g(q) = τ + τ_ext
 */
class FluidDynamicsCppNode : public rclcpp::Node
{
public:
  FluidDynamicsCppNode()
  : rclcpp::Node("fluid_dynamics_cpp_node")
  {
    urdf_path_ = this->declare_parameter<std::string>("urdf_path", "");
    bool use_free_flyer = this->declare_parameter<bool>("free_flyer", false);

    if (urdf_path_.empty())
    {
      throw std::runtime_error("参数 urdf_path 为空，无法加载模型。");
    }

    try
    {
      if (use_free_flyer)
      {
        pinocchio::urdf::buildModel(urdf_path_, pinocchio::JointModelFreeFlyer(), model_);
      }
      else
      {
        pinocchio::urdf::buildModel(urdf_path_, model_);
      }
    }
    catch (const std::exception & exc)
    {
      RCLCPP_FATAL(this->get_logger(), "加载 URDF 失败: %s", exc.what());
      throw;
    }

    data_ = pinocchio::Data(model_);
    q_ = Eigen::VectorXd::Zero(model_.nq);
    v_ = Eigen::VectorXd::Zero(model_.nv);
    a_ = Eigen::VectorXd::Zero(model_.nv);

    fluid_frames_ = this->declare_parameter<std::vector<std::string>>(
      "fluid_frames", std::vector<std::string>{"link_1", "link_2"});
    if (fluid_frames_.empty())
    {
      RCLCPP_WARN(this->get_logger(), "未配置 fluid_frames，不会施加流体外力。");
    }

    frame_ids_.reserve(fluid_frames_.size());
    for (const auto & name : fluid_frames_)
    {
      frame_ids_.push_back(model_.getFrameId(name));
    }

    auto v_current_world_vec = this->declare_parameter<std::vector<double>>(
      "v_current_world", std::vector<double>{0.0, 0.0, 0.0});
    if (v_current_world_vec.size() != 3)
    {
      throw std::runtime_error("v_current_world 需要 3 个元素。");
    }
    v_current_world_ = Eigen::Vector3d(
      v_current_world_vec[0], v_current_world_vec[1], v_current_world_vec[2]);

    rho_ = this->declare_parameter<double>("rho", 1000.0);
    default_area_ = this->declare_parameter<double>("default_area", 0.01);
    default_cd_lin_ = this->declare_parameter<double>("default_cd_lin", 0.0);
    default_cd_quad_ = this->declare_parameter<double>("default_cd_quad", 0.0);
    default_cd_ang_lin_ = this->declare_parameter<double>("default_cd_ang_lin", 0.1);

    for (const auto & frame : fluid_frames_)
    {
      FrameConfig cfg;
      cfg.area = this->declare_parameter<double>(frame + ".area", default_area_);
      cfg.cd_lin = this->declare_parameter<double>(frame + ".cd_lin", default_cd_lin_);
      cfg.cd_quad = this->declare_parameter<double>(frame + ".cd_quad", default_cd_quad_);
      cfg.cd_ang_lin = this->declare_parameter<double>(frame + ".cd_ang_lin", default_cd_ang_lin_);

      cfg.mu = this->declare_parameter<double>(frame + ".m_u", 0.089);
      cfg.mv = this->declare_parameter<double>(frame + ".m_v", 0.089);
      cfg.mr = this->declare_parameter<double>(frame + ".m_r", 0.16);

      cfg.dlu = this->declare_parameter<double>(frame + ".d_lu", 0.4);
      cfg.dlv = this->declare_parameter<double>(frame + ".d_lv", 3.0);
      cfg.dlr = this->declare_parameter<double>(frame + ".d_lr", 0.0);

      cfg.dnu = this->declare_parameter<double>(frame + ".d_nu", 12.9);
      cfg.dnv = this->declare_parameter<double>(frame + ".d_nv", 21.3);
      cfg.dnr = this->declare_parameter<double>(frame + ".d_nr", 0.0);

      std::vector<double> dist_added_vec = this->declare_parameter<std::vector<double>>(
        frame + ".dist_added", std::vector<double>{0.0, 0.0, 0.0});
      std::vector<double> dist_damp_vec = this->declare_parameter<std::vector<double>>(
        frame + ".dist_damp", std::vector<double>{0.0, 0.0, 0.0});

      if (dist_added_vec.size() == 3)
      {
        cfg.dist_added = Eigen::Vector3d(dist_added_vec[0], dist_added_vec[1], dist_added_vec[2]);
      }
      if (dist_damp_vec.size() == 3)
      {
        cfg.dist_damp = Eigen::Vector3d(dist_damp_vec[0], dist_damp_vec[1], dist_damp_vec[2]);
      }

      frame_configs_.emplace(frame, cfg);
    }

    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::SensorDataQoS(),
      std::bind(&FluidDynamicsCppNode::on_joint_state, this, std::placeholders::_1));
    tau_ext_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("tau_ext_cpp", 10);

    RCLCPP_INFO(this->get_logger(), "fluid_dynamics_cpp_node 已启动");
  }

private:
  void on_joint_state(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (msg->position.empty())
    {
      return;
    }

    double dt = 0.0;
    rclcpp::Time stamp = msg->header.stamp;
    if (stamp.nanoseconds() == 0)
    {
      stamp = this->get_clock()->now();
    }
    if (has_prev_time_)
    {
      dt = (stamp - prev_time_).seconds();
      if (dt <= 0.0)
      {
        dt = 0.0;
      }
    }
    prev_time_ = stamp;
    has_prev_time_ = true;

    const auto copy_to_eigen = [](const std::vector<double> & src, Eigen::VectorXd & dst) {
      auto n = std::min<std::size_t>(src.size(), static_cast<std::size_t>(dst.size()));
      for (std::size_t i = 0; i < n; ++i)
      {
        dst(static_cast<Eigen::Index>(dst.size() - n + i)) = src[src.size() - n + i];
      }
    };

    copy_to_eigen(msg->position, q_);
    if (!msg->velocity.empty())
    {
      copy_to_eigen(msg->velocity, v_);
    }
    else
    {
      v_.setZero();
    }
    a_.setZero();

    pinocchio::forwardKinematics(model_, data_, q_, v_, a_);
    pinocchio::updateFramePlacements(model_, data_);

    pinocchio::container::aligned_vector<pinocchio::Force> fext(model_.njoints, pinocchio::Force::Zero());

    for (std::size_t idx = 0; idx < fluid_frames_.size(); ++idx)
    {
      const auto & name = fluid_frames_[idx];
      FrameConfig & cfg = frame_configs_.at(name);
      pinocchio::FrameIndex fid = frame_ids_[idx];

      pinocchio::Motion Vw = pinocchio::getFrameVelocity(model_, data_, fid, pinocchio::ReferenceFrame::WORLD);
      Eigen::Vector3d v_lin = Vw.linear();
      Eigen::Vector3d v_ang = Vw.angular();

      Eigen::Vector3d v_rel_lin = v_lin - v_current_world_;
      Eigen::Vector3d v_rel_ang = v_ang;

      const pinocchio::SE3 & X_o_f = data_.oMf[fid];
      Eigen::Matrix3d R_of = X_o_f.rotation().transpose();
      Eigen::Vector3d body_linear = R_of * v_rel_lin;
      double u_b = body_linear.x();
      double v_b = body_linear.y();
      double r_b = v_rel_ang.z();

      double fD_u = cfg.dlu * u_b + cfg.dnu * std::abs(u_b) * u_b;
      double fD_v = cfg.dlv * v_b + cfg.dnv * std::abs(v_b) * v_b;
      double fD_r = cfg.dlr * r_b + cfg.dnr * std::abs(r_b) * r_b;

      double du_dt = 0.0;
      double dv_dt = 0.0;
      double dr_dt = 0.0;
      if (cfg.has_prev_body_vel && dt > 0.0)
      {
        du_dt = (u_b - cfg.prev_body_vel.x()) / dt;
        dv_dt = (v_b - cfg.prev_body_vel.y()) / dt;
        dr_dt = (r_b - cfg.prev_body_vel.z()) / dt;
      }
      cfg.prev_body_vel = Eigen::Vector3d(u_b, v_b, r_b);
      cfg.has_prev_body_vel = true;

      double fA_u = cfg.mu * du_dt;
      double fA_v = cfg.mv * dv_dt;
      double nA_r = cfg.mr * dr_dt;

      double speed = v_rel_lin.norm();
      Eigen::Vector3d f_quad_world = -0.5 * rho_ * cfg.cd_quad * cfg.area * speed * v_rel_lin;

      Eigen::Vector3d f_lin_body(fD_u + fA_u, fD_v + fA_v, 0.0);
      Eigen::Vector3d f_lin_world_from_body = X_o_f.rotation() * f_lin_body;
      Eigen::Vector3d force_world = f_lin_world_from_body + f_quad_world - X_o_f.rotation() * cfg.dist_added - X_o_f.rotation() * cfg.dist_damp;

      Eigen::Vector3d n_world = -cfg.cd_ang_lin * v_rel_ang + Eigen::Vector3d(0.0, 0.0, fD_r + nA_r);

      pinocchio::Force F_world(force_world, n_world);
      pinocchio::JointIndex parent = model_.frames[fid].parentJoint;
      const pinocchio::SE3 & X_o_parent = data_.oMi[parent];
      pinocchio::Force F_parent = X_o_parent.actInv(F_world);
      fext[parent] += F_parent;
    }

    Eigen::VectorXd tau_with = pinocchio::rnea(model_, data_, q_, v_, a_, fext);
    Eigen::VectorXd tau_no = pinocchio::rnea(model_, data_, q_, v_, a_);
    Eigen::VectorXd tau_ext = tau_with - tau_no;

    if (!verify_logged_)
    {
      pinocchio::crba(model_, data_, q_);
      pinocchio::computeCoriolisMatrix(model_, data_, q_, v_);
      pinocchio::computeGeneralizedGravity(model_, data_, q_);
      Eigen::VectorXd lhs = data_.M * a_ + data_.C * v_ + data_.g;
      Eigen::VectorXd residual = lhs - (tau_with - tau_ext);
      RCLCPP_INFO(
        this->get_logger(),
        "验证公式: ||M*qdd + C*v + g - (tau - tau_ext)|| = %.3e",
        residual.norm());
      verify_logged_ = true;
    }

    std_msgs::msg::Float64MultiArray msg_out;
    msg_out.data.resize(static_cast<std::size_t>(tau_ext.size()));
    for (Eigen::Index i = 0; i < tau_ext.size(); ++i)
    {
      msg_out.data[static_cast<std::size_t>(i)] = tau_ext(i);
    }
    tau_ext_pub_->publish(msg_out);
  }

  std::string urdf_path_;
  std::vector<std::string> fluid_frames_;
  std::vector<pinocchio::FrameIndex> frame_ids_;

  double rho_{1000.0};
  double default_area_{0.01};
  double default_cd_lin_{0.0};
  double default_cd_quad_{0.0};
  double default_cd_ang_lin_{0.1};

  Eigen::Vector3d v_current_world_{Eigen::Vector3d::Zero()};

  pinocchio::Model model_;
  pinocchio::Data data_;
  Eigen::VectorXd q_;
  Eigen::VectorXd v_;
  Eigen::VectorXd a_;

  std::unordered_map<std::string, FrameConfig> frame_configs_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr tau_ext_pub_;

  rclcpp::Time prev_time_;
  bool has_prev_time_{false};
  bool verify_logged_{false};
};
}  // namespace pin_ros2_example

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try
  {
    auto node = std::make_shared<pin_ros2_example::FluidDynamicsCppNode>();
    rclcpp::spin(node);
  }
  catch (const std::exception & exc)
  {
    RCLCPP_FATAL(rclcpp::get_logger("fluid_dynamics_cpp_node"), "节点初始化失败: %s", exc.what());
    return EXIT_FAILURE;
  }
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}

