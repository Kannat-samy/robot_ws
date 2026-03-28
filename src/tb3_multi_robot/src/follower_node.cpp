#include <cmath>
#include <memory>
#include <string>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using std::placeholders::_1;

enum Phase { PHASE_1, PHASE_2 };

class SmartFollower : public rclcpp::Node
{
public:
  SmartFollower() : Node("follower_node")
  {
    leader_odom_topic_   = declare_parameter("leader_odom",    "/robot1/odom");
    follower_odom_topic_ = declare_parameter("follower_odom",  "/robot2/odom");
    cmd_vel_topic_       = declare_parameter("cmd_vel",        "/robot2/cmd_vel");
    leader_plan_topic_   = declare_parameter("leader_plan",    "/robot1/plan");
    follower_scan_topic_ = declare_parameter("follower_scan",  "/robot2/scan");

    target_distance_ = declare_parameter("target_distance", 1.0);
    kp_dist_         = declare_parameter("kp_dist",  1.5);
    kp_yaw_          = declare_parameter("kp_yaw",   2.0);
    lookahead_dist_  = declare_parameter("lookahead_dist", 0.5);
    arrive_thresh_   = declare_parameter("arrive_thresh",  0.15); // Phase1 → Phase2
    safety_dist_     = declare_parameter("safety_dist",    0.35);
    max_lin_vel_     = declare_parameter("max_lin_vel", 0.5);
    max_ang_vel_     = declare_parameter("max_ang_vel", 2.0);

    leader_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      leader_odom_topic_, 10, std::bind(&SmartFollower::leaderCb, this, _1));

    follower_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      follower_odom_topic_, 10, std::bind(&SmartFollower::followerCb, this, _1));

    plan_sub_ = create_subscription<nav_msgs::msg::Path>(
      leader_plan_topic_, 10, std::bind(&SmartFollower::planCb, this, _1));

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      follower_scan_topic_, rclcpp::SensorDataQoS(),
      std::bind(&SmartFollower::scanCb, this, _1));

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);

    timer_ = create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&SmartFollower::controlLoop, this));

    RCLCPP_INFO(get_logger(), "Follower ready.");
  }

private:
  // --------------------------------------------------------------------------
  void leaderCb(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    leader_ = *msg;
    leader_ok_ = true;
    if (!distance_initialized_ && follower_ok_) {
      double dx = msg->pose.pose.position.x - follower_.pose.pose.position.x;
      double dy = msg->pose.pose.position.y - follower_.pose.pose.position.y;
      target_distance_ = std::hypot(dx, dy);
      RCLCPP_INFO(get_logger(), "Distance calibrated: %.3f m", target_distance_);
      distance_initialized_ = true;
    }
  }

  void followerCb(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    follower_ = *msg;
    follower_ok_ = true;
  }

  void scanCb(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    scan_     = *msg;
    scan_ok_  = true;
  }

  // Called whenever Nav2 publishes a (re)plan for the leader
  void planCb(const nav_msgs::msg::Path::SharedPtr msg)
  {
    if (msg->poses.size() < 2) return;

    bool new_goal = false;

    if (!plan_ok_) {
      new_goal = true; // first goal ever
    } else {
      // Detect a real new goal by comparing the goal position (last pose)
      const auto & old_g = leader_plan_.poses.back().pose.position;
      const auto & new_g = msg->poses.back().pose.position;
      if (std::hypot(new_g.x - old_g.x, new_g.y - old_g.y) > 0.3) {
        new_goal = true;
      }
    }

    leader_plan_ = *msg;
    plan_ok_     = true;

    if (new_goal && leader_ok_) {
      // Record the leader's position RIGHT NOW as the Phase 1 target
      phase1_x_ = leader_.pose.pose.position.x;
      phase1_y_ = leader_.pose.pose.position.y;
      phase_    = PHASE_1;
      RCLCPP_INFO(get_logger(),
        "New goal → Phase 1: go to leader's current pos (%.2f, %.2f)",
        phase1_x_, phase1_y_);
    }
  }

  // --------------------------------------------------------------------------
  int findClosestIdx(double x, double y)
  {
    const auto & poses = leader_plan_.poses;
    int best = 0; double best_d = 1e9;
    for (int i = 0; i < (int)poses.size(); i++) {
      double d = std::hypot(poses[i].pose.position.x - x,
                            poses[i].pose.position.y - y);
      if (d < best_d) { best_d = d; best = i; }
    }
    return best;
  }

  // Returns the minimum range in a ±30° forward arc of the follower's laser
  double frontObstacleDist()
  {
    if (!scan_ok_) return 1e9;
    const auto & r  = scan_.ranges;
    const int    n  = r.size();
    const int fwd   = (int)((0.0f - scan_.angle_min) / scan_.angle_increment);
    const int half  = (int)(0.524f / scan_.angle_increment); // 30°
    double min_d    = 1e9;
    for (int i = fwd - half; i <= fwd + half; i++) {
      int idx = ((i % n) + n) % n;
      float d = r[idx];
      if (d > scan_.range_min && d < scan_.range_max)
        min_d = std::min(min_d, (double)d);
    }
    return min_d;
  }

  // --------------------------------------------------------------------------
  void controlLoop()
  {
    if (!leader_ok_ || !follower_ok_ || !plan_ok_) return;

    const double xF   = follower_.pose.pose.position.x;
    const double yF   = follower_.pose.pose.position.y;
    const double yawF = getYaw(follower_.pose.pose.orientation);

    double tx, ty;    // heading target
    double v_cmd = 0.0;

    // ======================================================================
    //  PHASE 1 — Go to where the leader was when the goal was set
    // ======================================================================
    if (phase_ == PHASE_1) {
      double dx = phase1_x_ - xF;
      double dy = phase1_y_ - yF;
      double d  = std::hypot(dx, dy);

      if (d < arrive_thresh_) {
        phase_ = PHASE_2;
        RCLCPP_INFO(get_logger(), "Phase 1 done → Phase 2: following the plan.");
        // Fall through to Phase 2 immediately
      } else {
        tx    = phase1_x_;
        ty    = phase1_y_;
        v_cmd = std::min(max_lin_vel_, kp_dist_ * d);

        double target_yaw = std::atan2(ty - yF, tx - xF);
        double yaw_err    = target_yaw - yawF;
        while (yaw_err >  M_PI) yaw_err -= 2.0 * M_PI;
        while (yaw_err < -M_PI) yaw_err += 2.0 * M_PI;

        if (std::abs(yaw_err) > 1.0) v_cmd *= 0.3;

        v_cmd = applyObstacleSafety(v_cmd);

        geometry_msgs::msg::Twist cmd;
        cmd.linear.x  = clamp(v_cmd,             0.0, max_lin_vel_);
        cmd.angular.z = clamp(kp_yaw_ * yaw_err, -max_ang_vel_, max_ang_vel_);
        cmd_pub_->publish(cmd);
        return;
      }
    }

    // ======================================================================
    //  PHASE 2 — Pure-pursuit along the leader's Nav2 plan
    //
    //  Find the follower's closest waypoint on the plan, then look ahead
    //  lookahead_dist_ along the plan to get a smooth steering target.
    //  Speed is controlled by the distance to the leader (spring).
    // ======================================================================
    {
      const auto & poses = leader_plan_.poses;

      int idx_f = findClosestIdx(xF, yF);

      // Walk forward from idx_f by lookahead_dist_ to get the carrot
      double acc = 0.0;
      tx = poses.back().pose.position.x;
      ty = poses.back().pose.position.y;

      for (int i = idx_f; i < (int)poses.size() - 1; i++) {
        double seg_dx = poses[i+1].pose.position.x - poses[i].pose.position.x;
        double seg_dy = poses[i+1].pose.position.y - poses[i].pose.position.y;
        double seg    = std::hypot(seg_dx, seg_dy);
        if (acc + seg >= lookahead_dist_) {
          double t = (lookahead_dist_ - acc) / seg;
          tx = poses[i].pose.position.x + t * seg_dx;
          ty = poses[i].pose.position.y + t * seg_dy;
          break;
        }
        acc += seg;
      }

      const double xL         = leader_.pose.pose.position.x;
      const double yL         = leader_.pose.pose.position.y;
      const double dist_leader = std::hypot(xL - xF, yL - yF);
      const double dist_error  = dist_leader - target_distance_;
      const double v_leader    = leader_.twist.twist.linear.x;

      v_cmd = v_leader + kp_dist_ * dist_error;

      double target_yaw = std::atan2(ty - yF, tx - xF);
      double yaw_err    = target_yaw - yawF;
      while (yaw_err >  M_PI) yaw_err -= 2.0 * M_PI;
      while (yaw_err < -M_PI) yaw_err += 2.0 * M_PI;

      if (std::abs(yaw_err) > 1.0) v_cmd *= 0.3;

      if (std::abs(v_leader) < 0.01 && std::abs(dist_error) < 0.05) v_cmd = 0.0;

      v_cmd = applyObstacleSafety(v_cmd);

      geometry_msgs::msg::Twist cmd;
      cmd.linear.x  = clamp(v_cmd,             -max_lin_vel_, max_lin_vel_);
      cmd.angular.z = clamp(kp_yaw_ * yaw_err, -max_ang_vel_, max_ang_vel_);
      cmd_pub_->publish(cmd);
    }
  }

  // Scale down / stop if obstacle too close in forward arc
  double applyObstacleSafety(double v_cmd)
  {
    if (v_cmd <= 0.0) return v_cmd;
    double d = frontObstacleDist();
    if (d < safety_dist_)           return 0.0;
    if (d < safety_dist_ * 2.0)     return v_cmd * (d - safety_dist_) / safety_dist_;
    return v_cmd;
  }

  // --------------------------------------------------------------------------
  double getYaw(const geometry_msgs::msg::Quaternion & q)
  {
    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
  }

  double clamp(double v, double lo, double hi)
  {
    return std::max(lo, std::min(v, hi));
  }

  // --------------------------------------------------------------------------
  std::string leader_odom_topic_, follower_odom_topic_, cmd_vel_topic_,
              leader_plan_topic_, follower_scan_topic_;

  double target_distance_, kp_dist_, kp_yaw_, lookahead_dist_,
         arrive_thresh_, safety_dist_, max_lin_vel_, max_ang_vel_;
  bool   distance_initialized_{false};

  nav_msgs::msg::Odometry     leader_, follower_;
  nav_msgs::msg::Path         leader_plan_;
  sensor_msgs::msg::LaserScan scan_;

  bool leader_ok_{false}, follower_ok_{false}, plan_ok_{false}, scan_ok_{false};

  Phase  phase_{PHASE_1};
  double phase1_x_{0.0}, phase1_y_{0.0};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr     leader_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr     follower_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr         plan_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr      cmd_pub_;
  rclcpp::TimerBase::SharedPtr                                 timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SmartFollower>());
  rclcpp::shutdown();
  return 0;
}
