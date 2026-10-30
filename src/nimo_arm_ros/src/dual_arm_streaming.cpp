#include "nimo_arm_ros/dual_arm_node.hpp"

#include <algorithm>

namespace nimo_arm_ros
{

double DualArmNode::ScaleJogJoint(int joint, double direction) const
  {
    if (joint < 0 || joint >= nimo_arm::kDof || !has_presets_) {
      return 0.0;
    }
    const double d = std::clamp(direction, -1.0, 1.0);
    return d * cfg_.jog_limit.joint.max_v(joint);
  }

double DualArmNode::ScaleJogCart(uint8_t axis, double direction) const
  {
    if (!has_presets_) {
      return 0.0;
    }
    const double d = std::clamp(direction, -1.0, 1.0);
    if (axis <= 2) {
      return d * cfg_.jog_limit.cart.max_line_v;
    }
    return d * cfg_.jog_limit.cart.max_angle_v;
  }

void DualArmNode::BindArmTriggers(const std::string & side, bool left)
  {
    auto add_trig = [this, left](const std::string & name, auto fn) {
        arm_services_.push_back(
          create_service<std_srvs::srv::Trigger>(
            ns_ + "/" + name,
            [this, left, fn](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
              std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
                if (!RequireInit(res)) {return;}
                nimo_arm::Arm & arm = left ? dual_.Left() : dual_.Right();
                const bool ok = fn(arm);
                res->success = ok;
                res->message = ok ? "ok" : RejectMsg(arm);
              }));
      };

    add_trig(side + "/enable", [](nimo_arm::Arm & a) {
        return a.SetEnable(nimo_arm::EnableMode::Enable);
      });
    add_trig(side + "/disable", [](nimo_arm::Arm & a) {
        return a.SetEnable(nimo_arm::EnableMode::Disable);
      });
    add_trig(side + "/stop", [](nimo_arm::Arm & a) {return a.Stop();});
    add_trig(side + "/estop", [](nimo_arm::Arm & a) {return a.EStop();});
    add_trig(side + "/clear_error", [](nimo_arm::Arm & a) {return a.ClearError();});
    add_trig(side + "/reset_estop", [](nimo_arm::Arm & a) {return a.ResetEStop();});

    arm_services_.push_back(
      create_service<nimo_arm_msgs::srv::SetControlMode>(
        ns_ + "/" + side + "/set_control_mode",
        [this, left](const std::shared_ptr<nimo_arm_msgs::srv::SetControlMode::Request> req,
          std::shared_ptr<nimo_arm_msgs::srv::SetControlMode::Response> res) {
            if (!initialized_) {
              res->success = false;
              res->message = "not initialized";
              return;
            }
            nimo_arm::Arm & arm = left ? dual_.Left() : dual_.Right();
            const bool ok = arm.SetControlMode(ToControlMode(req->mode));
            res->success = ok;
            res->message = ok ? "ok" : RejectMsg(arm);
          }));

    arm_services_.push_back(
      create_service<nimo_arm_msgs::srv::SetBrake>(
        ns_ + "/" + side + "/set_brake",
        [this, left](const std::shared_ptr<nimo_arm_msgs::srv::SetBrake::Request> req,
          std::shared_ptr<nimo_arm_msgs::srv::SetBrake::Response> res) {
            if (!initialized_) {
              res->success = false;
              res->message = "not initialized";
              return;
            }
            nimo_arm::Arm & arm = left ? dual_.Left() : dual_.Right();
            const bool ok = arm.SetBrake(ToBrake(req->command));
            res->success = ok;
            res->message = ok ? "ok" : RejectMsg(arm);
          }));

    arm_services_.push_back(
      create_service<nimo_arm_msgs::srv::SetJointImpKd>(
        ns_ + "/" + side + "/set_joint_imp_kd",
        [this, left](const std::shared_ptr<nimo_arm_msgs::srv::SetJointImpKd::Request> req,
          std::shared_ptr<nimo_arm_msgs::srv::SetJointImpKd::Response> res) {
            if (!initialized_) {
              res->success = false;
              res->message = "not initialized";
              return;
            }
            nimo_arm::Arm & arm = left ? dual_.Left() : dual_.Right();
            arm.SetJointImpKd(ToV7Seq(req->k), ToV7Seq(req->d));
            res->success = true;
            res->message = "ok";
          }));

    arm_services_.push_back(
      create_service<nimo_arm_msgs::srv::SetCartImpKd>(
        ns_ + "/" + side + "/set_cart_imp_kd",
        [this, left](const std::shared_ptr<nimo_arm_msgs::srv::SetCartImpKd::Request> req,
          std::shared_ptr<nimo_arm_msgs::srv::SetCartImpKd::Response> res) {
            if (!initialized_) {
              res->success = false;
              res->message = "not initialized";
              return;
            }
            nimo_arm::Arm & arm = left ? dual_.Left() : dual_.Right();
            arm.SetCartImpKd(ToV7Seq(req->k), ToV7Seq(req->d));
            res->success = true;
            res->message = "ok";
          }));
  }

void DualArmNode::BindStreaming(const std::string & side, bool left)
  {
    auto sub_sj = create_subscription<sensor_msgs::msg::JointState>(
      ns_ + "/" + side + "/servo_j", 10,
      [this, left](const sensor_msgs::msg::JointState::SharedPtr msg) {
        // Low-level stream: same Hermite H as ServoJ teleop; not gated by mode.
        if (!initialized_ || traj_replaying_.load() || msg->position.size() < 7) {return;}
        nimo_arm::Arm & arm = left ? dual_.Left() : dual_.Right();
        arm.ServoJ(ToV7Seq(msg->position), servoj_interp_time_s_);
      });
    stream_subs_.push_back(sub_sj);

    auto sub_sp = create_subscription<geometry_msgs::msg::PoseStamped>(
      ns_ + "/" + side + "/servo_p", 10,
      [this, left](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        if (!initialized_) {return;}
        nimo_arm::Arm & arm = left ? dual_.Left() : dual_.Right();
        arm.ServoP(ToPose(msg->pose));
      });
    stream_subs_.push_back(sub_sp);

    auto sub_pico = create_subscription<nimo_arm_msgs::msg::ServoPByPico>(
      ns_ + "/" + side + "/servo_p_by_pico", 10,
      [this, left](const nimo_arm_msgs::msg::ServoPByPico::SharedPtr msg) {
        if (!initialized_ || traj_replaying_.load()) {return;}
        const bool run = teleop_motion_enabled_.load() && msg->is_run;
        nimo_arm::Arm & arm = left ? dual_.Left() : dual_.Right();
        arm.ServoPByPico(ToPose(msg->pose), run);
      });
    stream_subs_.push_back(sub_pico);

    auto sub_jogj = create_subscription<nimo_arm_msgs::msg::JogJoint>(
      ns_ + "/" + side + "/jog_j", 10,
      [this, left](const nimo_arm_msgs::msg::JogJoint::SharedPtr msg) {
        if (!initialized_) {return;}
        nimo_arm::Arm & arm = left ? dual_.Left() : dual_.Right();
        arm.JogJ(msg->joint, ScaleJogJoint(msg->joint, msg->dq));
      });
    stream_subs_.push_back(sub_jogj);

    auto sub_jogc = create_subscription<nimo_arm_msgs::msg::JogCart>(
      ns_ + "/" + side + "/jog_cart", 10,
      [this, left](const nimo_arm_msgs::msg::JogCart::SharedPtr msg) {
        if (!initialized_) {return;}
        nimo_arm::Arm & arm = left ? dual_.Left() : dual_.Right();
        arm.JogCart(ToJogCartAxis(msg->axis), ScaleJogCart(msg->axis, msg->rate));
      });
    stream_subs_.push_back(sub_jogc);
  }

}  // namespace nimo_arm_ros
