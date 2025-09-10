#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

class LidarMapNode : public rclcpp::Node {
public:
  LidarMapNode(const std::string& robot_name) 
    : Node(robot_name + "_global_map_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_) {

      // Params
      this->declare_parameter<std::string>("map_frame", "world");
      this->declare_parameter<double>("resolution", 1.0f);
      this->declare_parameter<double>("tf_timeout_sec", 0.05);

      // Construct dynamic topics using robot_name
      pointcloud_topic_ = "/" + robot_name + "/ouster/points";
      // pointcloud_topic_ = "/" + robot_name + "/ouster/semantic_points";
      // robot_frame_ = robot_name + "_base_link";

      // Retrieve params
      map_frame_ = this->get_parameter("map_frame").as_string();
      resolution_ = this->get_parameter("resolution").as_double();
      tf_timeout_sec_   = this->get_parameter("tf_timeout_sec").as_double();
      
      // Subscriber and publishers
      pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
          pointcloud_topic_, rclcpp::SensorDataQoS(),
          std::bind(&LidarMapNode::pointCloudCallback, this, std::placeholders::_1));
      global_map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/" + robot_name + "/global_map", 100);
      local_map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/" + robot_name + "/local_map", 1);
  }

private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg) {
    try {

      geometry_msgs::msg::TransformStamped transform_stamped = tf_buffer_.lookupTransform(
          map_frame_, cloud_msg->header.frame_id,
          rclcpp::Time(cloud_msg->header.stamp),
          rclcpp::Duration::from_seconds(tf_timeout_sec_)
      );

      // // Get the transform from the robot frame to the map frame
      // geometry_msgs::msg::TransformStamped transform_stamped =
      //     tf_buffer_.lookupTransform(map_frame_, cloud_msg->header.frame_id, tf2::TimePointZero);

      // Convert PointCloud2 to PCL
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
      pcl::fromROSMsg(*cloud_msg, *cloud);

      // Transform the cloud to the map frame
      Eigen::Affine3d transform = tf2::transformToEigen(transform_stamped.transform);
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZRGB>());
      pcl::transformPointCloud(*cloud, *cloud_transformed, transform);

      // Voxel Downsample
      pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter;
      voxel_filter.setInputCloud(cloud_transformed);
      voxel_filter.setLeafSize(resolution_, resolution_, resolution_); // Leaf size in meters
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZRGB>());
      voxel_filter.filter(*cloud_downsampled);

      // Aggregate the point cloud
      *aggregated_cloud_ += *cloud_downsampled;
      
      voxel_filter.setInputCloud(aggregated_cloud_);
      voxel_filter.setLeafSize(resolution_, resolution_, resolution_); // Leaf size in meters
      voxel_filter.filter(*aggregated_cloud_);

      // For all points: replace those with RGB (255, 0, 0) to RGB (128, 64, 128)
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr globalMap(new pcl::PointCloud<pcl::PointXYZRGB>());
      for (auto& pt : aggregated_cloud_->points) {
        if (pt.r == 255 && pt.g == 0 && pt.b == 0) {
          pt.r = 128;
          pt.g =  64;
          pt.b = 128;
        }

        if ((pt.r != 0   && pt.g != 100 && pt.b != 0) || 
            (pt.r != 220 && pt.g != 20  && pt.b != 60) ||
            (pt.r != 0   && pt.g != 100 && pt.b != 0)) 
        {
          globalMap->points.push_back(pt);
        }
      }

      publishMapAndPointCloud(globalMap);

      // LOCAL CLOUD
      double max_range_ = 1000.0; // radius of local map
      Eigen::Vector3f center = transform.translation().cast<float>();
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_sphere_filtered(new pcl::PointCloud<pcl::PointXYZRGB>());
      for (const auto& point : aggregated_cloud_->points) {
        Eigen::Vector3f pt(point.x, point.y, point.z);
        if ((pt - center).norm() <= max_range_) {
          cloud_sphere_filtered->points.push_back(point);
        }
      }
      cloud_sphere_filtered->width = cloud_sphere_filtered->points.size();
      cloud_sphere_filtered->height = 1;
      cloud_sphere_filtered->is_dense = true;
      pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter2;
      voxel_filter2.setInputCloud(cloud_sphere_filtered);
      publishLocalMap(cloud_sphere_filtered);

    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "Transform error: %s", ex.what());
    }
  }

  void publishMapAndPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr globalMap) {
    // Publish the aggregated point cloud
    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(*globalMap, cloud_msg);
    cloud_msg.header.stamp = this->get_clock()->now();
    cloud_msg.header.frame_id = map_frame_;
    global_map_pub_->publish(cloud_msg);
  }

  void publishLocalMap(pcl::PointCloud<pcl::PointXYZRGB>::Ptr localMap) {
    sensor_msgs::msg::PointCloud2 localMap_pc2;
    pcl::toROSMsg(*localMap, localMap_pc2);
    localMap_pc2.header.stamp = this->get_clock()->now();
    localMap_pc2.header.frame_id = "world";
    local_map_pub_->publish(localMap_pc2);
  }

  // Parameters
  std::string pointcloud_topic_;
  std::string map_frame_;
  // std::string robot_frame_;
  double resolution_;

  // ROS 2 components
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr global_map_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr local_map_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Transform listener
  double tf_timeout_sec_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // Global point cloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr aggregated_cloud_ = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  // if (argc < 2) {
  //   RCLCPP_ERROR(rclcpp::get_logger("global_map_node"), "Usage: lidar_map_node <robot_name>");
  //   return 1;
  // }

  std::string robot_name = argv[1];
  rclcpp::spin(std::make_shared<LidarMapNode>(robot_name));
  rclcpp::shutdown();
  return 0;
}