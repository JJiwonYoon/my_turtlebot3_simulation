#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/float32.hpp>
#include <cmath>
#include <vector>
#include <algorithm>

using std::placeholders::_1;

class Detection : public rclcpp::Node {
public:
    Detection() : Node("detection_node") {
        partition_count_ = 8;
        min_global_y_LiDAR_ = 200;
        global_x_LiDAR_ = 400; // mm
        global_y_LiDAR_ = 400; // mm
        added_range_ = static_cast<int>(2 * (global_x_LiDAR_ / partition_count_));

        lidar_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 1, std::bind(&Detection::LiDARCallback, this, _1));
        target_num_publisher_ = this->create_publisher<std_msgs::msg::Float32>("/LiDAR_target_num", 1);
    }

private:
    void LiDARCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        std::vector<int> partition_list(partition_count_, 0);
        std::vector<float> distance_data;
        int target_num = 0;

        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            float angle = msg->angle_increment * i;
            float x = -std::sin(angle) * msg->ranges[i] * 1000;
            float y = std::cos(angle) * msg->ranges[i] * 1000;

            if (std::abs(x) < global_x_LiDAR_ && y > min_global_y_LiDAR_ && y < global_y_LiDAR_) {
                for (int j = 0; j < partition_count_; ++j) {
                    if ((-global_x_LiDAR_ + j * added_range_) < x &&
                        x < (-global_x_LiDAR_ + (j + 1) * added_range_)) {
                        float distance = std::sqrt(x * x + y * y);
                        distance_data.push_back(distance);
                        partition_list[j] = distance_data.size();
                    }
                }
                auto max_partition = std::max_element(partition_list.begin(), partition_list.end());
                target_num = std::distance(partition_list.begin(), max_partition) + 1;
            }
        }

        auto float_msg = std_msgs::msg::Float32();
        float_msg.data = static_cast<float>(target_num);
        RCLCPP_INFO(this->get_logger(), "num: %d", target_num);
        target_num_publisher_->publish(float_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscription_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr target_num_publisher_;

    int partition_count_;
    int min_global_y_LiDAR_;
    int global_x_LiDAR_;
    int global_y_LiDAR_;
    int added_range_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Detection>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
