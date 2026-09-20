#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <pybind11/embed.h>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <Eigen/Dense>

#include <sstream>

using namespace std::chrono_literals;

struct odom_info {
  std::vector<double> position;
  std::vector<double> orientation;
  std::vector<double> linear_vel;
};

namespace py = pybind11;

class __attribute__((visibility("hidden"))) Racetrack {
public:
  Racetrack(const std::string& csv) {
    py::module mod = py::module::import("racetrack_utilities.racetrack_utilities");
    util_ = mod.attr("RacetrackUtilities")(csv);
  }

  Eigen::Vector2d to_frenet(double x, double y) {
    return util_.attr("convert_to_frenet")(x, y).cast<Eigen::Vector2d>();
  }
  Eigen::Vector2d to_cartesian(double s, double d) {
    return util_.attr("convert_to_cartesian")(s, d).cast<Eigen::Vector2d>();
  }
  bool in_bounds(double s, double d) {
    return util_.attr("in_bounds_frenet")(s, d).cast<bool>();
  }
private:
  py::object util_;
};


class SplineTracking : public rclcpp::Node
{
public:
  SplineTracking()
  : Node("ego_rrt_mppi")
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/ego_racecar/odom",  10, std::bind(&SplineTracking::ego_odom_sub, this, std::placeholders::_1));
    timer_ = this->create_wall_timer(std::chrono::milliseconds(200), std::bind(&SplineTracking::timer_cb, this));
  }

  void ego_odom_sub(const nav_msgs::msg::Odometry::SharedPtr msg) {
    ego_odom.position = {msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z};
    ego_odom.orientation = {msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z};
    ego_odom.linear_vel = {msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z};
    new_odom = true;
  }

  void generate_rrt_traj() {

  }

  void timer_cb() {
      if (new_odom) {
        Eigen::Vector2d frenet_state = rutil.to_frenet(ego_odom.position[0], ego_odom.position[1]);
        double s = frenet_state(0);
        double d = frenet_state(1);
        std::stringstream ss;
        ss << "Frenet state: s - " << s << ", d - " << d << "\n";
        RCLCPP_INFO(this->get_logger(), ss.str());
      }
  }
  
private:
  rclcpp::TimerBase::SharedPtr timer_;
  // subscribers and publishers
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  // ego odometry
  odom_info ego_odom;
  // racetrack utilities
  const std::string path = "/sim_ws/src/lidar_processing/scripts/Spielberg_map.csv";
  Racetrack rutil{path};

  bool new_odom = false;

};

int main(int argc, char * argv[])
{
  py::scoped_interpreter guard{};
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SplineTracking>());
  rclcpp::shutdown();
  return 0;
}