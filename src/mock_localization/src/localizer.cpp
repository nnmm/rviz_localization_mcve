#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

using namespace std::chrono_literals;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("localizer")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "lidar_1d",
      10,
      [this](sensor_msgs::msg::LaserScan::UniquePtr msg) {
        RCLCPP_INFO(get_logger(), "Received %f", msg->ranges.at(0));
        std::this_thread::sleep_for(20ms);
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = msg->header.stamp;
        tf.header.frame_id = "map";
        tf.child_frame_id = "base_link";
        tf.transform.rotation.x = 0.0;
        tf.transform.rotation.y = 0.0;
        tf.transform.rotation.z = 0.0;
        tf.transform.rotation.w = 1.0;
        tf2_msgs::msg::TFMessage tf_msg;
        tf_msg.transforms.push_back(tf);
        tf_publisher_->publish(tf_msg);
      });
    tf_publisher_ = create_publisher<tf2_msgs::msg::TFMessage>("/tf", 10);
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_publisher_{nullptr};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
