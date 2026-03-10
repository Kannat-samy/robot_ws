#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <deque>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point.hpp"

using std::placeholders::_1;

class SmartFollower : public rclcpp::Node
{
public:
  SmartFollower() : Node("follower_node")
  {
    // =================================================================================
    //                                PARAMÈTRES
    // =================================================================================
    
    leader_odom_topic_   = declare_parameter("leader_odom", "/robot1/odom");
    follower_odom_topic_ = declare_parameter("follower_odom", "/robot2/odom");
    cmd_vel_topic_       = declare_parameter("cmd_vel", "/robot2/cmd_vel");

    target_distance_ = declare_parameter("target_distance", 1.0);

    // KP_DIST : Ressort (Accélérateur/Frein)
    kp_dist_ = declare_parameter("kp_dist", 2.0); 

    // KP_YAW : Volant
    kp_yaw_  = declare_parameter("kp_yaw", 3.0);

    // Lookahead : Regarder devant sur le chemin
    lookahead_dist_ = declare_parameter("lookahead_dist", 0.4);

    max_lin_vel_ = declare_parameter("max_lin_vel", 0.5);
    max_ang_vel_ = declare_parameter("max_ang_vel", 2.0);

    // =================================================================================
    //                                ROS SETUP
    // =================================================================================
    
    leader_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      leader_odom_topic_, 10, std::bind(&SmartFollower::leaderCb, this, _1));

    follower_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      follower_odom_topic_, 10, std::bind(&SmartFollower::followerCb, this, _1));

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);

    timer_ = create_wall_timer(
      std::chrono::milliseconds(20),
      std::bind(&SmartFollower::controlLoop, this));

    RCLCPP_INFO(get_logger(), "--- MODE SMART LINK : ANTI-BOUCLE ACTIVE ---");
  }

private:
  void leaderCb(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    leader_ = *msg;
    leader_ok_ = true;
    
    geometry_msgs::msg::Point current_pos = msg->pose.pose.position;
    
    // --- 1. DETECTION DE BOUCLE (LOOP PRUNING) ---
    // Si le leader repasse près d'un ANCIEN point de son historique,
    // on considère que c'est un croisement/demi-tour.
    // ON SUPPRIME la boucle pour que le follower coupe tout droit ("Short-circuit").
    
    if (!path_.empty()) {
        // On cherche si on est proche d'un point passé (mais pas les 20 derniers cm pour pas détecter soi-même)
        int intersection_idx = -1;
        
        // On parcourt l'historique du début jusqu'à "récemment"
        for (size_t i = 0; i < path_.size(); i++) {
            double d = std::hypot(current_pos.x - path_[i].x, current_pos.y - path_[i].y);
            
            // Si on recroise le chemin à moins de 20cm
            if (d < 0.20) {
                // Vérifier qu'on ne détecte pas juste le point précédent (faut qu'il y ait de la distance dans l'index)
                // Disons qu'il faut au moins 50 points d'écart ou 1m de chemin parcouru
                size_t dist_index = path_.size() - i;
                if (dist_index > 50) { 
                    intersection_idx = i;
                    break; // On a trouvé la coupure la plus vieille
                }
            }
        }

        // Si on a trouvé une intersection (le leader a croisé ses traces)
        if (intersection_idx != -1) {
            // ON EFFACE TOUT ce qu'il y a entre l'intersection et maintenant
            // Le chemin devient : [Début ... Intersection -> Point Actuel]
            // La boucle disparaît de la mémoire du follower.
            path_.erase(path_.begin() + intersection_idx + 1, path_.end());
            // RCLCPP_INFO(get_logger(), "BOUCLE DETECTEE ! CHEMIN SIMPLIFIE.");
        }
    }

    // --- 2. AJOUT DU POINT ---
    if (path_.empty()) {
        path_.push_back(current_pos);
    } else {
        double dx = current_pos.x - path_.back().x;
        double dy = current_pos.y - path_.back().y;
        if (std::hypot(dx, dy) > 0.05) { 
            path_.push_back(current_pos);
        }
    }
  }

  void followerCb(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    follower_ = *msg;
    follower_ok_ = true;
  }

  void controlLoop()
  {
    if (!leader_ok_ || !follower_ok_) return;

    double xF = follower_.pose.pose.position.x;
    double yF = follower_.pose.pose.position.y;
    double yawF = getYaw(follower_.pose.pose.orientation);

    double xL = leader_.pose.pose.position.x;
    double yL = leader_.pose.pose.position.y;

    // CALIBRAGE
    if (!distance_initialized_) {
      target_distance_ = std::hypot(xL - xF, yL - yF);
      RCLCPP_INFO(get_logger(), "DISTANCE PLANCHE : %.3f m", target_distance_);
      distance_initialized_ = true;
    }

    double current_dist = std::hypot(xL - xF, yL - yF);
    double dist_error = current_dist - target_distance_;
    
    // --- VITESSE (RESSORT) ---
    double v_leader = leader_.twist.twist.linear.x;
    double v_cmd = v_leader + (kp_dist_ * dist_error);

    // --- DIRECTION (VOLANT) ---
    double target_yaw = 0.0;
    
    // MODE PANIQUE : Si la distance est TROP GRANDE (> cible + 50cm)
    // C'est que le Follower s'est perdu dans le demi-tour.
    // -> ON OUBLIE LE CHEMIN, ON VISE LEADER DIRECTEMENT.
    bool panic_mode = (dist_error > 0.5);

    if (path_.empty() || v_cmd < -0.05 || panic_mode) {
        // Viser le leader directement
        target_yaw = std::atan2(yL - yF, xL - xF);
    } 
    else {
        // SUIVI DE CHEMIN NORMAL
        double min_dist = 1e9;
        size_t closest_idx = 0;
        
        // Optimisation recherche
        size_t start_search = (last_closest_idx_ > 20) ? last_closest_idx_ - 20 : 0;
        size_t end_search = std::min(path_.size(), last_closest_idx_ + 50);
        if(path_.size() < 50) { start_search = 0; end_search = path_.size(); }

        for (size_t i = start_search; i < end_search; i++) {
            double d = std::hypot(path_[i].x - xF, path_[i].y - yF);
            if (d < min_dist) {
                min_dist = d;
                closest_idx = i;
            }
        }
        last_closest_idx_ = closest_idx;

        // Lookahead
        geometry_msgs::msg::Point aim_point = path_.back();
        double dist_ahead = 0.0;
        for (size_t i = closest_idx; i < path_.size() - 1; i++) {
            dist_ahead += std::hypot(path_[i+1].x - path_[i].x, path_[i+1].y - path_[i].y);
            if (dist_ahead >= lookahead_dist_) {
                aim_point = path_[i];
                break;
            }
        }
        target_yaw = std::atan2(aim_point.y - yF, aim_point.x - xF);
    }

    // Calcul Erreur Angle
    double yaw_err = target_yaw - yawF;
    while (yaw_err > M_PI) yaw_err -= 2.0 * M_PI;
    while (yaw_err < -M_PI) yaw_err += 2.0 * M_PI;

    // Si on tourne FORT, on ralentit la vitesse linéaire pour pivoter proprement
    if (std::abs(yaw_err) > 1.0) { // Si > 60 degrés
         v_cmd *= 0.3; // On freine fort pour pivoter
    }

    double omega_cmd = kp_yaw_ * yaw_err;

    // SECURITE ARRET
    if (std::abs(v_leader) < 0.01 && std::abs(dist_error) < 0.03) {
        v_cmd = 0.0;
        omega_cmd = 0.0;
    }

    // Publication
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = clamp(v_cmd, -max_lin_vel_, max_lin_vel_);
    cmd.angular.z = clamp(omega_cmd, -max_ang_vel_, max_ang_vel_);
    cmd_pub_->publish(cmd);
  }

  // UTILS
  double getYaw(const geometry_msgs::msg::Quaternion &q) {
    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
  }

  double clamp(double v, double lo, double hi) {
    return std::max(lo, std::min(v, hi));
  }

  // Vars
  std::string leader_odom_topic_, follower_odom_topic_, cmd_vel_topic_;
  double target_distance_; 
  bool distance_initialized_{false};
  
  double kp_dist_, kp_yaw_, lookahead_dist_;
  double max_lin_vel_, max_ang_vel_;

  nav_msgs::msg::Odometry leader_, follower_;
  bool leader_ok_{false}, follower_ok_{false};

  std::vector<geometry_msgs::msg::Point> path_;
  size_t last_closest_idx_{0};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr leader_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr follower_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SmartFollower>());
  rclcpp::shutdown();
  return 0;
}