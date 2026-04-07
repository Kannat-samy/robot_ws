#include <cmath>
#include <vector>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

using std::placeholders::_1;

class LidarFollower : public rclcpp::Node
{
public:
  LidarFollower() : Node("follower_node")
  {
    scan_topic_   = declare_parameter("scan_topic",   "/robot2/scan");
    cmd_vel_topic_ = declare_parameter("cmd_vel",     "/robot2/cmd_vel");

    target_distance_ = declare_parameter("target_distance", 0.5);
    kp_dist_         = declare_parameter("kp_dist",         1.5);
    kp_yaw_          = declare_parameter("kp_yaw",          2.0);
    max_lin_vel_     = declare_parameter("max_lin_vel",      0.22);
    max_ang_vel_     = declare_parameter("max_ang_vel",      2.0);

    // Seuil de segmentation : deux points consécutifs du scan appartiennent
    // au même cluster s'ils sont à moins de cluster_tol_ mètres l'un de l'autre.
    cluster_tol_ = declare_parameter("cluster_tol", 0.15);

    // Distance maximale pour ignorer les objets trop loin (murs, etc.)
    max_range_ = declare_parameter("max_range", 3.5);

    auto qos = rclcpp::QoS(rclcpp::SensorDataQoS());
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, qos, std::bind(&LidarFollower::scanCb, this, _1));

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);

    RCLCPP_INFO(get_logger(), "LidarFollower prêt — scan: %s  cmd: %s",
      scan_topic_.c_str(), cmd_vel_topic_.c_str());
  }

private:
  // -------------------------------------------------------------------------
  void scanCb(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    // 1. Convertir les rayons valides en points (repère robot)
    struct Point2D { double x, y; };
    std::vector<Point2D> pts;
    pts.reserve(msg->ranges.size());

    float angle = msg->angle_min;
    for (float r : msg->ranges) {
      if (std::isfinite(r) && r > msg->range_min && r < max_range_) {
        pts.push_back({r * std::cos(angle), r * std::sin(angle)});
      } else {
        // On insère un point invalide pour conserver l'ordre angulaire
        // (utile pour la segmentation consécutive)
        pts.push_back({std::numeric_limits<double>::quiet_NaN(), 0.0});
      }
      angle += msg->angle_increment;
    }

    // 2. Segmentation en clusters (points consécutifs proches)
    // Un cluster est une liste d'indices de points valides.
    struct Cluster { std::vector<size_t> indices; double min_dist; };
    std::vector<Cluster> clusters;
    Cluster current;
    current.min_dist = std::numeric_limits<double>::max();

    for (size_t i = 0; i < pts.size(); ++i) {
      const auto & p = pts[i];
      if (!std::isfinite(p.x)) {
        // Point invalide : ferme le cluster en cours
        if (current.indices.size() >= 2) clusters.push_back(current);
        current = Cluster();
        current.min_dist = std::numeric_limits<double>::max();
        continue;
      }

      double dist = std::hypot(p.x, p.y);

      if (current.indices.empty()) {
        current.indices.push_back(i);
        current.min_dist = dist;
      } else {
        const auto & prev = pts[current.indices.back()];
        double seg = std::hypot(p.x - prev.x, p.y - prev.y);
        if (seg < cluster_tol_) {
          current.indices.push_back(i);
          if (dist < current.min_dist) current.min_dist = dist;
        } else {
          if (current.indices.size() >= 2) clusters.push_back(current);
          current = Cluster();
          current.indices.push_back(i);
          current.min_dist = dist;
        }
      }
    }
    if (current.indices.size() >= 2) clusters.push_back(current);

    if (clusters.empty()) {
      stopRobot();
      return;
    }

    // 3. Cluster le plus proche = robot1 (garanti par hypothèse)
    const Cluster * best = &clusters[0];
    for (const auto & c : clusters) {
      if (c.min_dist < best->min_dist) best = &c;
    }

    // 4. Centroïde du cluster
    double cx = 0.0, cy = 0.0;
    for (size_t idx : best->indices) { cx += pts[idx].x; cy += pts[idx].y; }
    cx /= best->indices.size();
    cy /= best->indices.size();
    double distance = std::hypot(cx, cy);

    // Calibration : on mémorise la distance du premier frame comme consigne
    if (!distance_calibrated_) {
      target_distance_ = distance;
      RCLCPP_INFO(get_logger(), "Distance initiale calibrée : %.3f m", target_distance_);
      distance_calibrated_ = true;
    }
    double angle_to_target = std::atan2(cy, cx);  // dans le repère robot

    // 6. PID distance + PID yaw
    double dist_error = distance - target_distance_;

    // Deadband : ne pas bouger pour de petites erreurs (bruit du scan)
    double v_cmd     = (std::abs(dist_error)      > 0.05) ? kp_dist_ * dist_error      : 0.0;
    double omega_cmd = (std::abs(angle_to_target) > 0.05) ? kp_yaw_  * angle_to_target : 0.0;

    // Réduction vitesse linéaire si grand écart angulaire
    if (std::abs(angle_to_target) > 0.5) {
      v_cmd *= 0.3;
    }

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
      "clusters=%zu  dist=%.3f  target=%.3f  err=%.3f  angle=%.3f  v=%.3f  w=%.3f",
      clusters.size(), distance, target_distance_, dist_error,
      angle_to_target, v_cmd, omega_cmd);

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x  = clamp(v_cmd,     -max_lin_vel_, max_lin_vel_);
    cmd.angular.z = clamp(omega_cmd, -max_ang_vel_, max_ang_vel_);
    cmd_pub_->publish(cmd);
  }

  // -------------------------------------------------------------------------
  void stopRobot()
  {
    cmd_pub_->publish(geometry_msgs::msg::Twist{});
  }

  double clamp(double v, double lo, double hi) {
    return std::max(lo, std::min(v, hi));
  }

  // Params
  std::string scan_topic_, cmd_vel_topic_;
  double target_distance_, kp_dist_, kp_yaw_;
  double max_lin_vel_, max_ang_vel_;
  double cluster_tol_, max_range_;

  // Calibration distance
  bool distance_calibrated_{false};

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarFollower>());
  rclcpp::shutdown();
  return 0;
}
