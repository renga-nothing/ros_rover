#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

class LidarIMUTransformNode : public rclcpp::Node
{
public:
    LidarIMUTransformNode() : Node("lidar_imu_tf_node")
    {
        // Subscribe to IMU data from LiDAR
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "/unilidar/imu", 10,  // ✅ Updated topic name
    std::bind(&LidarIMUTransformNode::imuCallback, this, std::placeholders::_1));

        // TF Broadcaster
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        RCLCPP_INFO(this->get_logger(), "✅ Lidar IMU TF Node Started. Waiting for IMU messages...");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

   void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    tf2::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = this->now();
    transform.header.frame_id = "map";  // ✅ This ensures "map" exists
    transform.child_frame_id = "unilidar_lidar";  // ✅ Correct child frame

    transform.transform.translation.x = 0.0;
    transform.transform.translation.y = 0.0;
    transform.transform.translation.z = 0.0;

    transform.transform.rotation.x = msg->orientation.x;
    transform.transform.rotation.y = msg->orientation.y;
    transform.transform.rotation.z = msg->orientation.z;
    transform.transform.rotation.w = msg->orientation.w;

    tf_broadcaster_->sendTransform(transform);

    RCLCPP_INFO(this->get_logger(), "✅ TF Updated: map -> unilidar_lidar | Roll: %.2f°, Pitch: %.2f°, Yaw: %.2f°", 
                roll * 180.0 / M_PI, pitch * 180.0 / M_PI, yaw * 180.0 / M_PI);
}


};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarIMUTransformNode>());
    rclcpp::shutdown();
    return 0;
}

