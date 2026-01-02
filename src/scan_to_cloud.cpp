#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "laser_geometry/laser_geometry.hpp"

class ScanToCloud : public rclcpp::Node
{
public:
    ScanToCloud()
    : Node("scan_to_cloud_node")
    {
        // Create subscriber
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&ScanToCloud::scanCallback, this, std::placeholders::_1));

        // Create publisher
        cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/pointcloud", 10);

        RCLCPP_INFO(this->get_logger(), "scan_to_cloud_node started.");
    }

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
    {
        sensor_msgs::msg::PointCloud2 cloud;

        // Convert LaserScan to PointCloud2
        projector_.projectLaser(*scan_msg, cloud);

        cloud.header.stamp = scan_msg->header.stamp;
        cloud.header.frame_id = scan_msg->header.frame_id;

        cloud_pub_->publish(cloud);
    }

    // Laser projection tool
    laser_geometry::LaserProjection projector_;

    // Publishers & subscribers
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ScanToCloud>());
    rclcpp::shutdown();
    return 0;
}
