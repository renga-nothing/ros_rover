#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <tf2/LinearMath/`.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <std_srvs/srv/trigger.hpp>
#include <ctime>
#include <filesystem>


class LidarMappingNode : public rclcpp::Node
{
public:
    LidarMappingNode() : Node("lidar_mapping_node")
    {
        lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/unilidar/cloud", rclcpp::SensorDataQoS(),
            std::bind(&LidarMappingNode::pointCloudCallback, this, std::placeholders::_1));

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/unilidar/imu", rclcpp::SensorDataQoS(),
            std::bind(&LidarMappingNode::imuCallback, this, std::placeholders::_1));

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        octomap_pub_ = this->create_publisher<octomap_msgs::msg::Octomap>("/octomap", 10);

        // ✅ Save OctoMap every 1 minute (changed from 5 minutes for quicker testing)
        save_timer_ = this->create_wall_timer(
            std::chrono::minutes(1),
            std::bind(&LidarMappingNode::saveOctoMap, this));

        // ✅ ROS2 service to manually save OctoMap
        save_service_ = this->create_service<std_srvs::srv::Trigger>(
            "save_octomap", 
            std::bind(&LidarMappingNode::manualSaveCallback, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "✅ Lidar Mapping Node Started - Move with LiDAR to Map the Environment.");
    }

    ~LidarMappingNode()
    {
        // ✅ Force save OctoMap before shutting down
        RCLCPP_INFO(this->get_logger(), "🛑 Node shutting down, saving final map...");
        saveOctoMap();
    }

    void saveOctoMap(); // ✅ Made public so `main()` can call it

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr octomap_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr save_timer_;  
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_service_;  

    octomap::OcTree octree_{0.05};  // ✅ OctoMap with 5cm resolution
    double roll_ = 0.0, pitch_ = 0.0, yaw_ = 0.0;
    bool map_updated_ = false;

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void manualSaveCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                            std::shared_ptr<std_srvs::srv::Trigger::Response> response);
};

void LidarMappingNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    tf2::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    tf2::Matrix3x3 m(q);
    m.getRPY(roll_, pitch_, yaw_);
}

void LidarMappingNode::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg, cloud);

    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(cloud.makeShared());
    voxel_filter.setLeafSize(0.05f, 0.05f, 0.05f);
    pcl::PointCloud<pcl::PointXYZ> filtered_cloud;
    voxel_filter.filter(filtered_cloud);

    tf2::Quaternion q;
    q.setRPY(roll_, pitch_, yaw_);
    tf2::Matrix3x3 rotation_matrix(q);

    for (auto &point : filtered_cloud.points)
    {
        tf2::Vector3 point_vector(point.x, point.y, point.z);
        tf2::Vector3 rotated_point = rotation_matrix * point_vector;

        octomap::point3d octo_point(rotated_point.x(), rotated_point.y(), rotated_point.z());
        octree_.updateNode(octo_point, true);
    }

    RCLCPP_INFO(this->get_logger(), "📌 Received new LiDAR data, processing point cloud...");
    map_updated_ = true;

    auto octomap_msg = std::make_shared<octomap_msgs::msg::Octomap>();
    octomap_msgs::binaryMapToMsg(octree_, *octomap_msg);
    octomap_pub_->publish(*octomap_msg);
}

void LidarMappingNode::saveOctoMap()
{
    if (!map_updated_)
    {
        RCLCPP_WARN(this->get_logger(), "⚠️ No new data received. Skipping OctoMap save.");
        return;
    }

    std::string home_dir = std::getenv("HOME");
    std::filesystem::path save_directory = home_dir + "/ros2_foxy/maps/";

    std::error_code ec;
    if (!std::filesystem::exists(save_directory))
    {
        if (!std::filesystem::create_directories(save_directory, ec))
        {
            RCLCPP_ERROR(this->get_logger(), "❌ Failed to create directory: %s (Error: %s)", 
                         save_directory.c_str(), ec.message().c_str());
            return;
        }
        RCLCPP_INFO(this->get_logger(), "📂 Created save directory: %s", save_directory.c_str());
    }

    std::ostringstream filename;
    auto now = std::chrono::system_clock::now();
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    std::tm *ltm = std::localtime(&now_time);

    filename << "map_" 
             << (1900 + ltm->tm_year) 
             << (1 + ltm->tm_mon) 
             << ltm->tm_mday << "_"
             << ltm->tm_hour << ltm->tm_min << ltm->tm_sec << ".bt";

    std::filesystem::path full_path = save_directory / filename.str();

    if (octree_.writeBinary(full_path.string()))
    {
        RCLCPP_INFO(this->get_logger(), "✅ OctoMap successfully saved at: %s", full_path.c_str());
        map_updated_ = false;
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "❌ Failed to save OctoMap at: %s", full_path.c_str());
    }
}

void LidarMappingNode::manualSaveCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    saveOctoMap();
    response->success = true;
    response->message = "OctoMap successfully saved.";
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarMappingNode>();
    rclcpp::spin(node);

    RCLCPP_INFO(node->get_logger(), "🛑 Node shutting down, saving final map...");
    node->saveOctoMap();

    rclcpp::shutdown();
    return 0;
}
