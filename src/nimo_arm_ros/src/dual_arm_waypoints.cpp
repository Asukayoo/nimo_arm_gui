#include "nimo_arm_ros/dual_arm_node.hpp"

#include <yaml-cpp/yaml.h>

using namespace std::chrono_literals;

namespace nimo_arm_ros
{

void DualArmNode::LoadWaypointsFile()
  {
    std::lock_guard<std::mutex> lock(wp_mutex_);
    waypoints_.clear();
    if (!fs::exists(waypoint_file_)) {return;}
    try {
      YAML::Node root = YAML::LoadFile(waypoint_file_);
      if (!root["waypoints"]) {return;}
      for (const auto & n : root["waypoints"]) {
        nimo_arm_msgs::msg::Waypoint wp;
        wp.name = n["name"].as<std::string>("");
        wp.note = n["note"].as<std::string>("");
        wp.has_left = n["has_left"].as<bool>(false);
        wp.has_right = n["has_right"].as<bool>(false);
        if (n["left_q"]) {
          for (int i = 0; i < 7; ++i) {
            wp.left_q[static_cast<size_t>(i)] = n["left_q"][i].as<double>(0.0);
          }
        }
        if (n["right_q"]) {
          for (int i = 0; i < 7; ++i) {
            wp.right_q[static_cast<size_t>(i)] = n["right_q"][i].as<double>(0.0);
          }
        }
        if (!wp.name.empty()) {
          waypoints_[wp.name] = wp;
        }
      }
    } catch (const std::exception & e) {
      RCLCPP_WARN(get_logger(), "waypoint load failed: %s", e.what());
    }
  }

bool DualArmNode::SaveWaypointsFile()
  {
    try {
      fs::create_directories(fs::path(waypoint_file_).parent_path());
      YAML::Node root;
      YAML::Node list(YAML::NodeType::Sequence);
      for (const auto & kv : waypoints_) {
        const auto & wp = kv.second;
        YAML::Node n;
        n["name"] = wp.name;
        n["note"] = wp.note;
        n["has_left"] = wp.has_left;
        n["has_right"] = wp.has_right;
        YAML::Node lq(YAML::NodeType::Sequence);
        YAML::Node rq(YAML::NodeType::Sequence);
        for (int i = 0; i < 7; ++i) {
          lq.push_back(wp.left_q[static_cast<size_t>(i)]);
          rq.push_back(wp.right_q[static_cast<size_t>(i)]);
        }
        n["left_q"] = lq;
        n["right_q"] = rq;
        list.push_back(n);
      }
      root["waypoints"] = list;
      std::ofstream ofs(waypoint_file_);
      ofs << root;
      return true;
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "waypoint save failed: %s", e.what());
      return false;
    }
  }

void DualArmNode::HandleWaypointSave(
    const std::shared_ptr<nimo_arm_msgs::srv::WaypointSave::Request> & req,
    const std::shared_ptr<nimo_arm_msgs::srv::WaypointSave::Response> & res)
  {
    if (!initialized_) {
      res->success = false;
      res->message = "not initialized";
      return;
    }
    if (req->name.empty()) {
      res->success = false;
      res->message = "empty name";
      return;
    }
    nimo_arm_msgs::msg::Waypoint wp;
    wp.name = req->name;
    wp.note = req->note;
    wp.has_left = req->capture_left;
    wp.has_right = req->capture_right;
    wp.stamp = now();
    if (req->capture_left) {
      nimo_arm::RobotState st{};
      dual_.Left().GetResp(st);
      FillArray(st.joint.q, wp.left_q.data());
    }
    if (req->capture_right) {
      nimo_arm::RobotState st{};
      dual_.Right().GetResp(st);
      FillArray(st.joint.q, wp.right_q.data());
    }
    {
      std::lock_guard<std::mutex> lock(wp_mutex_);
      waypoints_[wp.name] = wp;
      if (!SaveWaypointsFile()) {
        res->success = false;
        res->message = "disk write failed";
        return;
      }
    }
    res->success = true;
    res->message = "saved " + req->name;
  }

void DualArmNode::HandleWaypointDelete(
    const std::shared_ptr<nimo_arm_msgs::srv::WaypointDelete::Request> & req,
    const std::shared_ptr<nimo_arm_msgs::srv::WaypointDelete::Response> & res)
  {
    std::lock_guard<std::mutex> lock(wp_mutex_);
    if (!waypoints_.erase(req->name)) {
      res->success = false;
      res->message = "not found";
      return;
    }
    res->success = SaveWaypointsFile();
    res->message = res->success ? "deleted" : "disk write failed";
  }

void DualArmNode::HandleWaypointList(const std::shared_ptr<nimo_arm_msgs::srv::WaypointList::Response> & res)
  {
    std::lock_guard<std::mutex> lock(wp_mutex_);
    res->success = true;
    res->message = "ok";
    res->waypoints.clear();
    for (const auto & kv : waypoints_) {
      res->waypoints.push_back(kv.second);
    }
  }

void DualArmNode::HandleWaypointGoto(
    const std::shared_ptr<nimo_arm_msgs::srv::WaypointGoto::Request> & req,
    const std::shared_ptr<nimo_arm_msgs::srv::WaypointGoto::Response> & res)
  {
    if (!initialized_) {
      res->success = false;
      res->message = "not initialized";
      return;
    }
    nimo_arm_msgs::msg::Waypoint wp;
    {
      std::lock_guard<std::mutex> lock(wp_mutex_);
      auto it = waypoints_.find(req->name);
      if (it == waypoints_.end()) {
        res->success = false;
        res->message = "not found";
        return;
      }
      wp = it->second;
    }
    bool ok = true;
    std::string msg = "ok";
    if (req->move_left) {
      if (!wp.has_left) {
        ok = false;
        msg = "waypoint has no left";
      } else if (!dual_.Left().MovJ(ToV7(wp.left_q))) {
        ok = false;
        msg = "left: " + RejectMsg(dual_.Left());
      }
    }
    if (req->move_right) {
      if (!wp.has_right) {
        ok = false;
        msg = msg == "ok" ? "waypoint has no right" : msg;
      } else if (!dual_.Right().MovJ(ToV7(wp.right_q))) {
        ok = false;
        msg = msg == "ok" ? ("right: " + RejectMsg(dual_.Right())) : msg;
      }
    }
    res->success = ok;
    res->message = msg;
  }

}  // namespace nimo_arm_ros
