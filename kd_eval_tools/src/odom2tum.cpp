#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <fstream>
#include <iomanip>
#include <string>

// odom2tum: subscribe to a nav_msgs/Odometry topic and write poses to a TUM file
// ros2 run kd_eval_tools odom2tum --ros-args -p topic:=/kiss/odometry -p output:=/tmp/kiss.tum

class Odom2Tum : public rclcpp::Node {
public:
  Odom2Tum() : Node("odom2tum") {
    declare_parameter("topic",  std::string("/kiss/odometry"));
    declare_parameter("output", std::string("/tmp/odom.tum"));
    const auto topic  = get_parameter("topic").as_string();
    const auto output = get_parameter("output").as_string();
    _out.open(output);
    if (!_out) {
      RCLCPP_ERROR(get_logger(), "cannot open output file: %s", output.c_str());
      rclcpp::shutdown();
      return;
    }
    _out << std::fixed << std::setprecision(9);
    RCLCPP_INFO(get_logger(), "subscribing to %s -> %s", topic.c_str(), output.c_str());
    _sub = create_subscription<nav_msgs::msg::Odometry>(topic, 100,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        const double ts = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
        const auto& p = msg->pose.pose.position;
        const auto& q = msg->pose.pose.orientation;
        _out << ts << " " << p.x << " " << p.y << " " << p.z
             << " " << q.x << " " << q.y << " " << q.z << " " << q.w << "\n";
      });
  }
private:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _sub;
  std::ofstream _out;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Odom2Tum>());
  rclcpp::shutdown();
  return 0;
}
