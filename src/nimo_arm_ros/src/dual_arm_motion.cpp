#include "nimo_arm_ros/dual_arm_node.hpp"

using namespace std::chrono_literals;

namespace nimo_arm_ros
{

void DualArmNode::ExecuteMovJ(const std::shared_ptr<GoalHandleMovJ> gh)
  {
    const auto goal = gh->get_goal();
    auto result = std::make_shared<MovJ::Result>();
    if (!initialized_) {
      result->success = false;
      result->message = "not initialized";
      gh->abort(result);
      return;
    }

    bool ok = true;
    std::string msg = "ok";
    if (goal->move_left) {
      if (!dual_.Left().MovJ(ToV7Seq(goal->left_q))) {
        ok = false;
        msg = "left reject: " + RejectMsg(dual_.Left());
      }
    }
    if (goal->move_right) {
      if (!dual_.Right().MovJ(ToV7Seq(goal->right_q))) {
        ok = false;
        msg = msg == "ok" ? ("right reject: " + RejectMsg(dual_.Right()))
                          : (msg + "; right reject: " + RejectMsg(dual_.Right()));
      }
    }
    if (!ok) {
      result->success = false;
      result->message = msg;
      gh->abort(result);
      return;
    }

    while (rclcpp::ok()) {
      if (gh->is_canceling()) {
        dual_.Left().Stop();
        dual_.Right().Stop();
        result->success = false;
        result->message = "canceled";
        gh->canceled(result);
        return;
      }
      auto fb = std::make_shared<MovJ::Feedback>();
      fb->left_op_state = ToMsg(dual_.Left().GetOpState());
      fb->right_op_state = ToMsg(dual_.Right().GetOpState());
      fb->left_done = !goal->move_left || dual_.Left().IsMotionDone();
      fb->right_done = !goal->move_right || dual_.Right().IsMotionDone();
      gh->publish_feedback(fb);
      if (fb->left_done && fb->right_done) {
        result->success = true;
        result->message = "done";
        gh->succeed(result);
        return;
      }
      std::this_thread::sleep_for(20ms);
    }
  }

void DualArmNode::ExecuteMovL(const std::shared_ptr<GoalHandleMovL> gh)
  {
    const auto goal = gh->get_goal();
    auto result = std::make_shared<MovL::Result>();
    if (!initialized_) {
      result->success = false;
      result->message = "not initialized";
      gh->abort(result);
      return;
    }

    bool ok = true;
    std::string msg = "ok";
    if (goal->move_left) {
      if (!dual_.Left().MovL(ToPose(goal->left_pose))) {
        ok = false;
        msg = "left reject: " + RejectMsg(dual_.Left());
      }
    }
    if (goal->move_right) {
      if (!dual_.Right().MovL(ToPose(goal->right_pose))) {
        ok = false;
        msg = msg == "ok" ? ("right reject: " + RejectMsg(dual_.Right()))
                          : (msg + "; right reject: " + RejectMsg(dual_.Right()));
      }
    }
    if (!ok) {
      result->success = false;
      result->message = msg;
      gh->abort(result);
      return;
    }

    while (rclcpp::ok()) {
      if (gh->is_canceling()) {
        dual_.Left().Stop();
        dual_.Right().Stop();
        result->success = false;
        result->message = "canceled";
        gh->canceled(result);
        return;
      }
      auto fb = std::make_shared<MovL::Feedback>();
      fb->left_op_state = ToMsg(dual_.Left().GetOpState());
      fb->right_op_state = ToMsg(dual_.Right().GetOpState());
      fb->left_done = !goal->move_left || dual_.Left().IsMotionDone();
      fb->right_done = !goal->move_right || dual_.Right().IsMotionDone();
      gh->publish_feedback(fb);
      if (fb->left_done && fb->right_done) {
        result->success = true;
        result->message = "done";
        gh->succeed(result);
        return;
      }
      std::this_thread::sleep_for(20ms);
    }
  }

}  // namespace nimo_arm_ros
