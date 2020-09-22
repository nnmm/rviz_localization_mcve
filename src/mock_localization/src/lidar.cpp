#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std::chrono_literals;

class LidarNode : public rclcpp::Node
{
public:
  LidarNode()
  : Node("lidar"), count_(0)
  {
    publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("lidar_1d", 10);
    auto timer_callback =
      [this]() -> void {
        auto message = sensor_msgs::msg::LaserScan();
        message.header.stamp = this->now();
        message.header.frame_id = "base_link";
		    message.angle_increment = 0.0f;
		    message.time_increment = 0.5f / 2;
		    message.range_min = 0.0f;
		    message.range_max = 100.0f;

		    message.scan_time = 0.5;

        auto range = 10.0f - static_cast<float>(this->count_) / 10.0f;
        message.ranges.push_back(range);
        message.intensities.push_back(100);
        message.ranges.push_back(range);
        message.intensities.push_back(100);
        RCLCPP_INFO(get_logger(), "Sending %f", range);
        this->count_ = (this->count_ + 1) % 1000;
        this->publisher_->publish(message);
      };
    timer_ = this->create_wall_timer(500ms, timer_callback);
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarNode>());
  rclcpp::shutdown();
  return 0;
}