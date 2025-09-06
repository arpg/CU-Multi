// file: lidar_map_node.cpp

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

#include <Eigen/Geometry>

#include <algorithm>
#include <limits>
#include <cstring>
#include <cmath>
#include <string>
#include <memory>

class LidarMapNode : public rclcpp::Node {
public:
  explicit LidarMapNode(const std::string& robot_name)
    : Node(robot_name + "_global_map_node"),
      tf_buffer_(this->get_clock()),
      tf_listener_(tf_buffer_) {

    // --- Parameters ---
    this->declare_parameter<std::string>("map_frame", "world");
    this->declare_parameter<double>("resolution", 1.0);
    this->declare_parameter<bool>("deskew_enabled", true);
    this->declare_parameter<std::string>("time_field", "t");
    this->declare_parameter<bool>("time_is_absolute", false); // false = per-scan relative
    this->declare_parameter<double>("time_scale_ns", 1.0);    // raw * time_scale_ns -> nanoseconds
    this->declare_parameter<double>("tf_timeout_sec", 0.05);
    this->declare_parameter<double>("local_radius", 100.0);

    // Topic & frames
    pointcloud_topic_ = "/" + robot_name + "/ouster/points";  // adjust if needed
    map_frame_        = this->get_parameter("map_frame").as_string();
    resolution_       = this->get_parameter("resolution").as_double();
    deskew_enabled_   = this->get_parameter("deskew_enabled").as_bool();
    time_field_       = this->get_parameter("time_field").as_string();
    time_is_absolute_ = this->get_parameter("time_is_absolute").as_bool();
    time_scale_ns_    = this->get_parameter("time_scale_ns").as_double();
    tf_timeout_sec_   = this->get_parameter("tf_timeout_sec").as_double();
    local_radius_     = this->get_parameter("local_radius").as_double();

    // ROS I/O
    using std::placeholders::_1;
    pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      pointcloud_topic_, rclcpp::SensorDataQoS(),
      std::bind(&LidarMapNode::pointCloudCallback, this, _1));

    global_map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/" + robot_name + "/global_map", 10);

    local_map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/" + robot_name + "/local_map", 10);

    RCLCPP_INFO(get_logger(),
      "LidarMapNode: topic='%s', map_frame='%s', resolution=%.3f, deskew=%s, time_field='%s'",
      pointcloud_topic_.c_str(), map_frame_.c_str(), resolution_,
      deskew_enabled_ ? "ON" : "OFF", time_field_.c_str());
  }

private:
  // ---------- Deskew helpers ----------
  struct TimeFieldInfo {
    int offset = -1;
    uint8_t datatype = 0; // sensor_msgs::msg::PointField::*
    bool found = false;
  };

  static inline Eigen::Quaterniond quatSlerp(const Eigen::Quaterniond& q0,
                                             const Eigen::Quaterniond& q1,
                                             double u) {
    return q0.slerp(u, q1);
  }

  static inline Eigen::Affine3d interpSE3(const Eigen::Affine3d& T0,
                                          const Eigen::Affine3d& T1,
                                          double u) {
    const Eigen::Vector3d p0 = T0.translation();
    const Eigen::Vector3d p1 = T1.translation();
    const Eigen::Quaterniond q0(T0.linear());
    const Eigen::Quaterniond q1(T1.linear());

    Eigen::Vector3d p = (1.0 - u) * p0 + u * p1;
    Eigen::Quaterniond q = quatSlerp(q0.normalized(), q1.normalized(), u);

    Eigen::Affine3d Ti = Eigen::Affine3d::Identity();
    Ti.linear() = q.toRotationMatrix();
    Ti.translation() = p;
    return Ti;
  }

  TimeFieldInfo findTimeField(const sensor_msgs::msg::PointCloud2& cloud,
                              const std::string& name) const {
    TimeFieldInfo info;
    for (const auto& f : cloud.fields) {
      if (f.name == name) {
        info.offset = f.offset;
        info.datatype = f.datatype;
        info.found = true;
        break;
      }
    }
    return info;
  }

  double readNumericField(const sensor_msgs::msg::PointCloud2& cloud,
                          const TimeFieldInfo& f,
                          size_t point_index) const {
    const uint8_t* base = cloud.data.data() + point_index * cloud.point_step + f.offset;
    switch (f.datatype) {
      case sensor_msgs::msg::PointField::INT8: {
        int8_t v; std::memcpy(&v, base, 1); return static_cast<double>(v);
      }
      case sensor_msgs::msg::PointField::UINT8: {
        uint8_t v; std::memcpy(&v, base, 1); return static_cast<double>(v);
      }
      case sensor_msgs::msg::PointField::INT16: {
        int16_t v; std::memcpy(&v, base, 2); return static_cast<double>(v);
      }
      case sensor_msgs::msg::PointField::UINT16: {
        uint16_t v; std::memcpy(&v, base, 2); return static_cast<double>(v);
      }
      case sensor_msgs::msg::PointField::INT32: {
        int32_t v; std::memcpy(&v, base, 4); return static_cast<double>(v);
      }
      case sensor_msgs::msg::PointField::UINT32: {
        uint32_t v; std::memcpy(&v, base, 4); return static_cast<double>(v);
      }
      case sensor_msgs::msg::PointField::FLOAT32: {
        float v; std::memcpy(&v, base, 4); return static_cast<double>(v);
      }
      case sensor_msgs::msg::PointField::FLOAT64: {
        double v; std::memcpy(&v, base, 8); return v;
      }
      default:
        // Unknown/unsupported type; return NaN so caller can skip
        return std::numeric_limits<double>::quiet_NaN();
    }
  }

  // ---------- Core callback ----------
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg) {
    try {
      // Convert to PCL early (we’ll write deskewed points into a new cloud)
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZRGB>());
      pcl::fromROSMsg(*cloud_msg, *cloud_in);

      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_map(new pcl::PointCloud<pcl::PointXYZRGB>());
      Eigen::Affine3d T_center = Eigen::Affine3d::Identity();

      if (!deskew_enabled_) {
        // Fast path: single TF at header.stamp
        auto tf = tf_buffer_.lookupTransform(
          map_frame_, cloud_msg->header.frame_id,
          rclcpp::Time(cloud_msg->header.stamp),
          rclcpp::Duration::from_seconds(tf_timeout_sec_));

        T_center = tf2::transformToEigen(tf.transform);
        Eigen::Matrix4f Tfloat = T_center.matrix().cast<float>();
        pcl::transformPointCloud(*cloud_in, *cloud_map, Tfloat);
      } else {
        // Deskew path
        const size_t N = static_cast<size_t>(cloud_msg->width) * cloud_msg->height;
        if (N == 0) return;

        // Find time field
        TimeFieldInfo tf_info = findTimeField(*cloud_msg, time_field_);
        if (!tf_info.found) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Deskew enabled but time field '%s' not found. Falling back to single TF.",
            time_field_.c_str());

          auto tf = tf_buffer_.lookupTransform(
            map_frame_, cloud_msg->header.frame_id,
            rclcpp::Time(cloud_msg->header.stamp),
            rclcpp::Duration::from_seconds(tf_timeout_sec_));

          T_center = tf2::transformToEigen(tf.transform);
          Eigen::Matrix4f Tfloat = T_center.matrix().cast<float>();
          pcl::transformPointCloud(*cloud_in, *cloud_map, Tfloat);
        } else {
          // Compute raw range for normalization
          double raw_min = std::numeric_limits<double>::infinity();
          double raw_max = -std::numeric_limits<double>::infinity();
          for (size_t i = 0; i < N; ++i) {
            double r = readNumericField(*cloud_msg, tf_info, i);
            if (!std::isfinite(r)) continue;
            raw_min = std::min(raw_min, r);
            raw_max = std::max(raw_max, r);
          }

          // Degenerate -> fallback
          if (!std::isfinite(raw_min) || !std::isfinite(raw_max) || raw_max <= raw_min) {
            auto tf = tf_buffer_.lookupTransform(
              map_frame_, cloud_msg->header.frame_id,
              rclcpp::Time(cloud_msg->header.stamp),
              rclcpp::Duration::from_seconds(tf_timeout_sec_));

            T_center = tf2::transformToEigen(tf.transform);
            Eigen::Matrix4f Tfloat = T_center.matrix().cast<float>();
            pcl::transformPointCloud(*cloud_in, *cloud_map, Tfloat);
          } else {
            // Define TF sampling times
            rclcpp::Time hdr_time(cloud_msg->header.stamp);
            rclcpp::Time t_start, t_end;

            if (time_is_absolute_) {
              // raw_min/raw_max are absolute nanoseconds since epoch
              // rclcpp::Time constructor accepts nanoseconds.
              t_start = rclcpp::Time(static_cast<int64_t>(raw_min));
              t_end   = rclcpp::Time(static_cast<int64_t>(raw_max));
            } else {
              // Treat raw as relative offsets; scale -> nanoseconds
              const double span_ns = (raw_max - raw_min) * time_scale_ns_;
              const double half_span_s = 0.5 * span_ns * 1e-9;
              t_start = hdr_time - rclcpp::Duration::from_seconds(half_span_s);
              t_end   = hdr_time + rclcpp::Duration::from_seconds(half_span_s);
            }

            // Lookup TF at start/end
            auto Tstamped0 = tf_buffer_.lookupTransform(
              map_frame_, cloud_msg->header.frame_id, t_start,
              rclcpp::Duration::from_seconds(tf_timeout_sec_));
            auto Tstamped1 = tf_buffer_.lookupTransform(
              map_frame_, cloud_msg->header.frame_id, t_end,
              rclcpp::Duration::from_seconds(tf_timeout_sec_));

            Eigen::Affine3d T0 = tf2::transformToEigen(Tstamped0.transform);
            Eigen::Affine3d T1 = tf2::transformToEigen(Tstamped1.transform);
            T_center = T1; // for local map center we’ll just use end pose

            // Deskew per point
            cloud_map->points.resize(cloud_in->points.size());
            cloud_map->width = cloud_in->width;
            cloud_map->height = cloud_in->height;
            cloud_map->is_dense = cloud_in->is_dense;

            const double denom = (raw_max - raw_min);
            for (size_t i = 0; i < N; ++i) {
              const auto& pin = cloud_in->points[i];
              auto& pout = cloud_map->points[i];

              double r = readNumericField(*cloud_msg, tf_info, i);
              // Normalize to [0,1]
              double u = (denom > 0.0) ? ((r - raw_min) / denom) : 0.0;
              u = std::clamp(u, 0.0, 1.0);

              Eigen::Affine3d Ti = interpSE3(T0, T1, u);
              Eigen::Vector3d p(pin.x, pin.y, pin.z);
              p = Ti * p;

              pout.x = static_cast<float>(p.x());
              pout.y = static_cast<float>(p.y());
              pout.z = static_cast<float>(p.z());
              pout.r = pin.r; pout.g = pin.g; pout.b = pin.b;
            }
          }
        }
      }

      // Continue with your existing pipeline on cloud_map (now in map_frame_)
      processAndPublish(cloud_map, T_center.translation(), *cloud_msg);

    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "Transform error: %s", ex.what());
    }
  }

  // ---------- Post-processing + publishing ----------
  void processAndPublish(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_in_map,
                         const Eigen::Vector3d& center_map,
                         const sensor_msgs::msg::PointCloud2& src_header_like) {
    // Voxel Downsample
    pcl::VoxelGrid<pcl::PointXYZRGB> voxel;
    voxel.setLeafSize(static_cast<float>(resolution_),
                      static_cast<float>(resolution_),
                      static_cast<float>(resolution_));
    voxel.setInputCloud(cloud_in_map);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_down(new pcl::PointCloud<pcl::PointXYZRGB>());
    voxel.filter(*cloud_down);

    // Aggregate
    *aggregated_cloud_ += *cloud_down;

    // Re-voxelize the aggregate to keep it tidy
    voxel.setInputCloud(aggregated_cloud_);
    voxel.filter(*aggregated_cloud_);

    // Recolor / filter classes (preserving your logic)
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr globalMap(new pcl::PointCloud<pcl::PointXYZRGB>());
    globalMap->points.reserve(aggregated_cloud_->points.size());
    for (auto& pt : aggregated_cloud_->points) {
      if (pt.r == 255 && pt.g == 0 && pt.b == 0) {
        pt.r = 128; pt.g = 64; pt.b = 128;
      }
      if ((pt.r != 0   && pt.g != 100 && pt.b != 0) ||
          (pt.r != 220 && pt.g != 20  && pt.b != 60) ||
          (pt.r != 0   && pt.g != 100 && pt.b != 0)) {
        globalMap->points.push_back(pt);
      }
    }

    // Publish global map
    sensor_msgs::msg::PointCloud2 global_pc2;
    pcl::toROSMsg(*globalMap, global_pc2);
    global_pc2.header = src_header_like.header; // keep original stamp
    global_pc2.header.frame_id = map_frame_;
    global_map_pub_->publish(global_pc2);

    // Build & publish local map (spherical crop around center_map)
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr local(new pcl::PointCloud<pcl::PointXYZRGB>());
    local->points.reserve(aggregated_cloud_->points.size());
    const Eigen::Vector3f center_f = center_map.cast<float>();
    for (const auto& p : aggregated_cloud_->points) {
      Eigen::Vector3f v(p.x, p.y, p.z);
      if ((v - center_f).norm() <= static_cast<float>(local_radius_)) {
        local->points.push_back(p);
      }
    }
    local->width = local->points.size();
    local->height = 1;
    local->is_dense = true;

    sensor_msgs::msg::PointCloud2 local_pc2;
    pcl::toROSMsg(*local, local_pc2);
    local_pc2.header = src_header_like.header; // keep original stamp
    local_pc2.header.frame_id = map_frame_;
    local_map_pub_->publish(local_pc2);
  }

  // ---------- Members ----------
  // Parameters / settings
  std::string pointcloud_topic_;
  std::string map_frame_;
  double resolution_{1.0};
  bool deskew_enabled_{true};
  std::string time_field_{"t"};
  bool time_is_absolute_{false};
  double time_scale_ns_{1.0};
  double tf_timeout_sec_{0.05};
  double local_radius_{100.0};

  // ROS 2 I/O
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr global_map_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr local_map_pub_;

  // TF
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // Aggregated global cloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr aggregated_cloud_{
    new pcl::PointCloud<pcl::PointXYZRGB>()
  };
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  if (argc < 2) {
    RCLCPP_ERROR(rclcpp::get_logger("global_map_node"),
                 "Usage: lidar_map_node <robot_name>");
    return 1;
  }

  std::string robot_name = argv[1];
  rclcpp::spin(std::make_shared<LidarMapNode>(robot_name));
  rclcpp::shutdown();
  return 0;
}
