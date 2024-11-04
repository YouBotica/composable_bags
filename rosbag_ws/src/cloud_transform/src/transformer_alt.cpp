#define PCL_NO_PRECOMPILE

#include <string> 
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/exceptions.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/transforms.hpp"
#include "pcl/point_cloud.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/filters/approximate_voxel_grid.h"
#include "pcl/octree/octree_pointcloud_voxelcentroid.h"
#include "pcl/segmentation/extract_clusters.h"
#include "pcl/common/pca.h"
#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include "pcl/impl/instantiate.hpp"

#include "pcl/segmentation/sac_segmentation.h"
#include "pcl/sample_consensus/ransac.h"

#include "autoware_auto_perception_msgs/msg/bounding_box.hpp"
#include "autoware_auto_perception_msgs/msg/bounding_box_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"



#include "blackandgold_msgs/msg/error_report.hpp"

using namespace std::chrono_literals;

struct EIGEN_ALIGN16 PointIrisCustom {
  PCL_ADD_POINT4D;                     // Adds x, y, z with float32 (matches datatype 7)
  float reflectance;                   // float32
  uint8_t return_index;                // uint8_t (datatype 2)
  uint8_t last_return_index;           // uint8_t (datatype 2)
  uint8_t sensor_id;                   // uint8_t (datatype 2)
  float azimuth;                       // float32
  float elevation;                     // float32
  float depth;                         // float32
  uint16_t line_index;                 // uint16_t (datatype 4)
  uint8_t frame_index;                 // uint8_t (datatype 2)
  uint8_t detector_site_id;            // uint8_t (datatype 2)
  uint8_t scan_checkpoint;             // uint8_t (datatype 2)
  uint8_t existence_probability_percent; // uint8_t (datatype 2)
  uint8_t data_qualifier;              // uint8_t (datatype 2)
  uint8_t blockage_level;              // uint8_t (datatype 2)
  // int timestamp;                  // Datatype 2, count 8 -> uint64_t (unsigned 64-bit)
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW      // Ensures proper memory alignment
} EIGEN_ALIGN16;                       // Make sure the point structure is 16-byte aligned

// Register the new point type with PCL
POINT_CLOUD_REGISTER_POINT_STRUCT(PointIrisCustom,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, reflectance, reflectance)
                                  (uint8_t, return_index, return_index)
                                  (uint8_t, last_return_index, last_return_index)
                                  (uint8_t, sensor_id, sensor_id)
                                  (float, azimuth, azimuth)
                                  (float, elevation, elevation)
                                  (float, depth, depth)
                                  (uint16_t, line_index, line_index)
                                  (uint8_t, frame_index, frame_index)
                                  (uint8_t, detector_site_id, detector_site_id)
                                  (uint8_t, scan_checkpoint, scan_checkpoint)
                                  (uint8_t, existence_probability_percent, existence_probability_percent)
                                  (uint8_t, data_qualifier, data_qualifier)
                                  (uint8_t, blockage_level, blockage_level)
                                  // (int, timestamp, timestamp)
)

class CloudTransformerAlt : public rclcpp::Node
{
  public:
    CloudTransformerAlt(): Node("alt_cloud_transform", rclcpp::NodeOptions().use_intra_process_comms(true))
    {
      // First, determine if the pointcloud requires fusion, or it is already fused or single
      this->declare_parameter<bool>("is_fused", false);
      this->get_parameter("is_fused", is_fused);

      this->declare_parameter<std::string>("target_frame", "center_of_gravity");
      this->get_parameter("target_frame", target_frame);

      this->declare_parameter<int>("downsampling_leaf", 1);
      this->get_parameter("downsampling_leaf", leaf_);
      this->declare_parameter<float>("z_axis_min", -1.0);
      this->get_parameter("z_axis_min", z_axis_min_);
      this->declare_parameter<float>("z_axis_max", 3.0);
      this->get_parameter("z_axis_max", z_axis_max_);
      this->declare_parameter<float>("voxel_leafx", 0.3);
      this->get_parameter("voxel_leafx", voxel_leafx);
      this->declare_parameter<float>("voxel_leafy", 0.3);
      this->get_parameter("voxel_leafy", voxel_leafy);
      this->declare_parameter<float>("voxel_leafz", 0.3);
      this->get_parameter("voxel_leafz", voxel_leafz);
      this->declare_parameter<int>("min_voxel_points", 10);
      this->get_parameter("min_voxel_points", min_voxel_points);
      this->declare_parameter<float>("octree_resolution", 0.1);
      this->get_parameter("octree_resolution", octree_resolution_);
      this->declare_parameter<float>("cluster_tolerance", 0.2);
      this->get_parameter("cluster_tolerance", cluster_tolerance_);
      this->declare_parameter<int>("min_cluster_size", 100);
      this->get_parameter("min_cluster_size", min_cluster_size_);
      this->declare_parameter<int>("max_cluster_size", 25000);
      this->get_parameter("max_cluster_size", max_cluster_size_);
      this->declare_parameter<int>("existence_prob_threshold", 130);
      this->get_parameter("existence_prob_threshold", existence_prob_threshold_);
      // this->declare_parameter<float>("pitch_offset", 0.0);
      // this->get_parameter("pitch_offset", pitch_offset_);

      // declare params for the different lidar frames
      if (!is_fused){ // We are receiving multiple pointclouds that must be fused
        this->declare_parameter<std::string>("front_lidar_source_frame", "luminar_front");
        this->declare_parameter<std::string>("left_lidar_source_frame", "luminar_left");
        this->declare_parameter<std::string>("right_lidar_source_frame", "luminar_right");

        // Lidar topic names
        this->declare_parameter<std::string>("front_lidar_topic", "/perception/luminar_front/points");
        this->declare_parameter<std::string>("left_lidar_topic", "/perception/luminar_left/points");
        this->declare_parameter<std::string>("right_lidar_topic", "/perception/luminar_right/points");

        // get values for the various parameters
        this->get_parameter("front_lidar_source_frame", front_lidar_source_frame);
        this->get_parameter("left_lidar_source_frame", left_lidar_source_frame);
        this->get_parameter("right_lidar_source_frame", right_lidar_source_frame);

        this->get_parameter("front_lidar_topic", front_lidar_topic);
        this->get_parameter("left_lidar_topic", left_lidar_topic);
        this->get_parameter("right_lidar_topic", right_lidar_topic);

        // timer = this->create_wall_timer(34ms, std::bind(&CloudTransformerAlt::timer_callback, this));

        // sleep(3);
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
        sub_front_point_cloud = this->create_subscription<sensor_msgs::msg::PointCloud2>(front_lidar_topic, qos, std::bind(&CloudTransformerAlt::receiveFrontPointCloud, this, std::placeholders::_1));
        sub_left_point_cloud = this->create_subscription<sensor_msgs::msg::PointCloud2>(left_lidar_topic, qos, std::bind(&CloudTransformerAlt::receiveLeftPointCloud, this, std::placeholders::_1));
        sub_right_point_cloud = this->create_subscription<sensor_msgs::msg::PointCloud2>(right_lidar_topic, qos, std::bind(&CloudTransformerAlt::receiveRightPointCloud, this, std::placeholders::_1));


      } else{ // We are only receiving one pointcloud
      this->declare_parameter<std::string>("single_lidar_topic", "/luminar_points");
        unique_lidar_source = this->get_parameter("single_lidar_topic").as_string();
        auto qos1 = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
        sub_single_point_cloud = this->create_subscription<sensor_msgs::msg::PointCloud2>(unique_lidar_source, qos1, std::bind(&CloudTransformerAlt::receivePointCloud, this, std::placeholders::_1));
      }


      // initialize a QOS object for pubs and subs
      auto qos = rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 1));
      qos.best_effort();

      pub_tf_pts = this->create_publisher<sensor_msgs::msg::PointCloud2>("luminar_points/fused", 10);//, rclcpp::QoS(10));
      pub_error_msgs = this->create_publisher<blackandgold_msgs::msg::ErrorReport>("/vehicle/errors", 10);//, rclcpp::QoS(10));


      tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
      tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
      tf_buffer_front = std::make_unique<tf2_ros::Buffer>(this->get_clock());
      tf_buffer_left = std::make_unique<tf2_ros::Buffer>(this->get_clock());
      tf_buffer_right = std::make_unique<tf2_ros::Buffer>(this->get_clock());
      transform_listener_front = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_front);
      transform_listener_left = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_left);
      transform_listener_right = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_right);
      
    }

  private:

    // sensor_msgs::msg::PointCloud2 pclToROSMsg(const pcl::PointCloud<pcl::PointXYZI>& cloud,
    //                                         const std::string& frame_id = "map") {
    //   sensor_msgs::msg::PointCloud2 cloud_ROS;
    //   pcl::toROSMsg(cloud, cloud_ROS);
    //   cloud_ROS.header.frame_id = frame_id;
    //   return cloud_ROS;
    // }

    void timer_callback()
    {
      auto start_timer_callback = std::chrono::high_resolution_clock::now(); // pre

      // get parameters for dynamic reconfiguring:
      this->get_parameter("downsampling_leaf", leaf_);
      this->get_parameter("z_axis_min", z_axis_min_);
      this->get_parameter("z_axis_max", z_axis_max_);
      this->get_parameter("voxel_leafx", voxel_leafx);
      this->get_parameter("voxel_leafy", voxel_leafy);
      this->get_parameter("voxel_leafz", voxel_leafz);
      this->get_parameter("min_voxel_points", min_voxel_points);
      this->get_parameter("octree_resolution", octree_resolution_);
      this->get_parameter("cluster_tolerance", cluster_tolerance_);
      this->get_parameter("min_cluster_size", min_cluster_size_);
      this->get_parameter("max_cluster_size", max_cluster_size_);



      // if (latest_update == "front") {transformed_pc_front = this->transformed_pc_front;} 

      // Transform pointclouds to target frame (cog):

      auto start_fusion = std::chrono::high_resolution_clock::now();

      sensor_msgs::msg::PointCloud2 fused_pc_1;
      fused_pc_1.header.frame_id = target_frame; //"center_of_gravity";
      bool fusion_1 = pcl::concatenatePointCloud(this->transformed_pc_front, this->transformed_pc_left, fused_pc_1);

      sensor_msgs::msg::PointCloud2 fused_pc_2;
      fused_pc_2.header.frame_id = target_frame; //"center_of_gravity";
      bool fusion_2 = pcl::concatenatePointCloud(fused_pc_1, this->transformed_pc_right, fused_pc_2);

      auto end_fusion = std::chrono::high_resolution_clock::now();
      std::chrono::duration<double> elapsed_fusion = end_fusion - start_fusion;
      RCLCPP_DEBUG(this->get_logger(), "Time taken to fuse point clouds: %f seconds", elapsed_fusion.count());


      if (fused_pc_2.data.size() <= 0) 
      {
        blackandgold_msgs::msg::ErrorReport error_msg;
        error_msg.description = "No pointcloud received from lidar front, left or right";
        error_msg.severity = 3; // No lidar data, the vehicle is blind
        error_msg.module = "Perception";
        error_msg.origin = "cloud_transform";
        error_msg.lifetime = 1;
        pub_error_msgs->publish(error_msg);
        return; 
      }

      builtin_interfaces::msg::Time latest_stamp;
      if (latest_update == "front") {latest_stamp =  transformed_pc_front.header.stamp;}
      else if (latest_update == "left"){latest_stamp =  transformed_pc_left.header.stamp;}
      else if (latest_update == "right"){latest_stamp =  transformed_pc_left.header.stamp;}
      else { 
        RCLCPP_ERROR(this->get_logger(), "Unknown source, algorithm is probably not correctly initialized, last update from: '%s'", latest_update);
        return;
      }

      // // Downsample:
      auto start_downsampling = std::chrono::high_resolution_clock::now();

      sensor_msgs::msg::PointCloud2::UniquePtr downsampledFusedCloud(new sensor_msgs::msg::PointCloud2());  
      // *downsampledFusedCloud = fused_pc_2;
      *downsampledFusedCloud = voxelizePointCloud(fused_pc_2);
      downsampledFusedCloud->header = this->transformed_pc_front.header;
      downsampledFusedCloud->header.frame_id = target_frame;

      auto end_downsampling = std::chrono::high_resolution_clock::now();
      std::chrono::duration<double> elapsed_downsampling = end_downsampling - start_downsampling;
      RCLCPP_DEBUG(this->get_logger(), "Time taken to downsample point cloud: %f seconds", elapsed_downsampling.count());

      auto end_timer_callback = std::chrono::high_resolution_clock::now();
      std::chrono::duration<double> elapsed_timer_callback = end_timer_callback - start_timer_callback;

      RCLCPP_INFO(this->get_logger(), "Time taken to execute timer callback: %f seconds", elapsed_timer_callback.count());

      auto start_publishing = std::chrono::high_resolution_clock::now();
      pub_tf_pts->publish(*downsampledFusedCloud);
      auto end_publishing = std::chrono::high_resolution_clock::now();
      std::chrono::duration<double> elapsed_publishing = end_publishing - start_publishing;
      RCLCPP_INFO(this->get_logger(), "Time taken to publish point cloud: %f seconds", elapsed_publishing.count());
    }
   
    void receiveFrontPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr pcl_msg)
    { 
      // Length of pointcloud:
      auto length = pcl_msg->width * pcl_msg->height; 
      RCLCPP_DEBUG(this->get_logger(), "Length: '%i'", length);

      if (length <= 0) 
      {
        blackandgold_msgs::msg::ErrorReport error_msg;
        error_msg.description = "No pointcloud received from lidar front";
        error_msg.severity = 3; // No lidar data, the vehicle is blind
        error_msg.module = "Perception";
        error_msg.origin = "cloud_transform";
        error_msg.lifetime = 1;
        pub_error_msgs->publish(error_msg);
      }

      std::string fromFrameRel = front_lidar_source_frame.c_str();
      std::string toFrameRel = target_frame; //"center_of_gravity";

      // Downsample:
      // sensor_msgs::msg::PointCloud2 downsampledFrontCloud;
      // downsampledFrontCloud = voxelizePointCloud(*pcl_msg);
      // downsampledFrontCloud.header = pcl_msg->header;


      geometry_msgs::msg::TransformStamped transformStamped;
      try {
        transformStamped = tf_buffer_front->lookupTransform(toFrameRel, fromFrameRel, tf2::TimePointZero);
      }
      catch (const tf2::TransformException& ex){
        RCLCPP_ERROR(this->get_logger(), "TF2 lookup failed: %s", ex.what());
        return;
      }


      // this->pitch_offset_ = this->get_parameter("pitch_offset").as_double()*3.14159/180.0;  

      // geometry_msgs::msg::Quaternion desired_rotation = eulerToQuaternion(0.0, 0.0, 0.0); // (0.0, this->pitch_offset_, 0.0);

      // transformStamped.transform.rotation.x = desired_rotation.x; //0.01945;
      // transformStamped.transform.rotation.y = 0.0;
      // transformStamped.transform.rotation.z = 0.0;
      // transformStamped.transform.rotation.w = desired_rotation.w; // 0.99981;

      // // Print rotation components:
      // RCLCPP_DEBUG(this->get_logger(), "Rotation x: %f", transformStamped.transform.rotation.x);
      // RCLCPP_DEBUG(this->get_logger(), "Rotation y: %f", transformStamped.transform.rotation.y);
      // RCLCPP_DEBUG(this->get_logger(), "Rotation z: %f", transformStamped.transform.rotation.z);
      // RCLCPP_DEBUG(this->get_logger(), "Rotation w: %f", transformStamped.transform.rotation.w);

      
      
      auto start = std::chrono::high_resolution_clock::now();
      pcl_ros::transformPointCloud(target_frame, transformStamped, *pcl_msg, this->transformed_pc_front);
      auto end = std::chrono::high_resolution_clock::now();
      std::chrono::duration<double> elapsed = end - start;
      RCLCPP_DEBUG(this->get_logger(), "Time taken to transform point cloud: %f seconds", elapsed.count());
      
      latest_update = "front";
      
      // Attach the timer callback to the front pointcloud callback:
      // TODO: Should probably add some sort of safety check here for safety node
      this->timer_callback();
      return;
    }

    geometry_msgs::msg::Quaternion eulerToQuaternion(double roll, double pitch, double yaw) {
      geometry_msgs::msg::Quaternion q;
      tf2::Quaternion tf_q;
      tf_q.setRPY(roll, pitch, yaw);
      q.x = tf_q.x();
      q.y = tf_q.y();
      q.z = tf_q.z();
      q.w = tf_q.w();
      return q;
    }

    void receiveLeftPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr pcl_msg)
    { 
      // Length of pointcloud:
      int length = sizeof(pcl_msg->data)/sizeof(pcl_msg->data);

      if (length <= 0) 
      {
        blackandgold_msgs::msg::ErrorReport error_msg;
        error_msg.description = "No pointcloud received from lidar left";
        error_msg.module = "Perception";
        error_msg.severity = 3; // No lidar data, the vehicle is blind
        error_msg.origin = "cloud_transform";
        error_msg.lifetime = 1;
        pub_error_msgs->publish(error_msg);
      }

      std::string fromFrameRel = left_lidar_source_frame.c_str();
      std::string toFrameRel = target_frame;//"center_of_gravity";

      // Downsample:
      // sensor_msgs::msg::PointCloud2 downsampledLeftCloud;
      // downsampledLeftCloud = voxelizePointCloud(*pcl_msg);
      // downsampledLeftCloud = DownsamplePCL(*pcl_msg);
      // downsampledLeftCloud.header.frame_id = target_frame;
      // downsampledLeftCloud.header = pcl_msg->header;

      geometry_msgs::msg::TransformStamped transformStamped;
      try {
        transformStamped = tf_buffer_front->lookupTransform(toFrameRel, fromFrameRel, tf2::TimePointZero);
      }
      catch (const tf2::TransformException& ex){
        RCLCPP_ERROR(this->get_logger(), "TF2 lookup failed: %s", ex.what());
        return;
      }

      pcl_ros::transformPointCloud(target_frame, transformStamped, *pcl_msg, this->transformed_pc_left);
      latest_update = "left";
      return;
    }

    void receiveRightPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr pcl_msg)
    { 
      // Length of pointcloud:
      int length = sizeof(pcl_msg->data)/sizeof(pcl_msg->data);

      if (length <= 0) 
      {
        blackandgold_msgs::msg::ErrorReport error_msg;
        error_msg.description = "No pointcloud received from lidar right";
        error_msg.module = "Perception";
        error_msg.severity = 3; // No lidar data, the vehicle is blind
        error_msg.origin = "cloud_transform";
        error_msg.lifetime = 1;
        pub_error_msgs->publish(error_msg);
      }
      std::string fromFrameRel = right_lidar_source_frame.c_str();
      std::string toFrameRel = target_frame;//"center_of_gravity";

      // Downsample:
      // sensor_msgs::msg::PointCloud2 downsampledRightCloud;
      // downsampledRightCloud = voxelizePointCloud(*pcl_msg);
      // downsampledRightCloud = DownsamplePCL(*pcl_msg);
      // downsampledRightCloud.header = pcl_msg->header;    

      geometry_msgs::msg::TransformStamped transformStamped;

      try {
        transformStamped = tf_buffer_front->lookupTransform(toFrameRel, fromFrameRel, tf2::TimePointZero);
      }
      catch (const tf2::TransformException& ex){
        RCLCPP_ERROR(this->get_logger(), "TF2 lookup failed: %s", ex.what());
        return;
      }

      // Print x,y,z component of translation
      // print from frame to fram
      RCLCPP_DEBUG(this->get_logger(), "From frame: %s", fromFrameRel.c_str());
      RCLCPP_DEBUG(this->get_logger(), "To frame: %s", toFrameRel.c_str());
      RCLCPP_DEBUG(this->get_logger(), "Translation x: %f", transformStamped.transform.translation.x);
      RCLCPP_DEBUG(this->get_logger(), "Translation y: %f", transformStamped.transform.translation.y);
      RCLCPP_DEBUG(this->get_logger(), "Translation z: %f", transformStamped.transform.translation.z);

      this->transformed_pc_right = sensor_msgs::msg::PointCloud2();

      pcl_ros::transformPointCloud(target_frame, transformStamped, *pcl_msg, this->transformed_pc_right);
      latest_update = "right";
      return;
    }

    void receivePointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr pcl_msg)
    { 

      /*** Transform Pointcloud to target frame ***/
      std::string fromFrameRel = pcl_msg->header.frame_id;
      std::string toFrameRel = target_frame;
      geometry_msgs::msg::TransformStamped transformStamped;
      sensor_msgs::msg::PointCloud2 transformed_pc;


      try {
        transformStamped = tf_buffer->lookupTransform(fromFrameRel, toFrameRel, tf2::TimePointZero);
      } catch (const tf2::TransformException & ex) {
        RCLCPP_INFO(
          this->get_logger(), "Could not transform %s to %s: %s",
          toFrameRel, fromFrameRel, ex.what());
        return;
      }


      pcl_ros::transformPointCloud(toFrameRel, transformStamped, *pcl_msg, transformed_pc);

      // Downsample:
      transformed_pc.header.frame_id = target_frame;
      transformed_pc.header.stamp = pcl_msg->header.stamp;
      sensor_msgs::msg::PointCloud2 voxelized_transformed_pointcloud;
      // downsampledFusedPCL = DownsamplePCL(transformed_pc);
      pub_tf_pts->publish(transformed_pc);
    }
  
  // Create a function that voxelizes the pointcloud for downsampling
  sensor_msgs::msg::PointCloud2 voxelizePointCloud(const sensor_msgs::msg::PointCloud2& input_cloud) {
    pcl::PointCloud<PointIrisCustom>::Ptr pcl_cloud(new pcl::PointCloud<PointIrisCustom>());
    pcl::PointCloud<PointIrisCustom>::Ptr cloud_voxelized(new pcl::PointCloud<PointIrisCustom>());

    // Convert from ROS message to PCL point cloud
    pcl::fromROSMsg(input_cloud, *pcl_cloud);

    // Print existence probability of a random point:
    RCLCPP_DEBUG(this->get_logger(), "Existence_prob: %i", pcl_cloud->points[100].existence_probability_percent);

    this->existence_prob_threshold_ = this->get_parameter("existence_prob_threshold").as_int();

    // Filter points based on height and existence_probability thresholds
    pcl::PointCloud<PointIrisCustom>::Ptr cloud_filtered(new pcl::PointCloud<PointIrisCustom>());
    for (const auto& point : pcl_cloud->points) {
      if (point.z >= z_axis_min_ && point.z <= z_axis_max_ && point.existence_probability_percent >= existence_prob_threshold_) {
        cloud_filtered->points.push_back(point);
      }
    }

    // Perform octree filtering
    pcl::octree::OctreePointCloudVoxelCentroid<PointIrisCustom> octree(this->octree_resolution_);
    octree.setInputCloud(cloud_filtered);
    octree.addPointsFromInputCloud();

    // Collect the centroids into a vector
    std::vector<PointIrisCustom, Eigen::aligned_allocator<PointIrisCustom>> voxel_centroids;
    octree.getVoxelCentroids(voxel_centroids);

    // Convert the vector to a PointCloud
    for (const auto& point : voxel_centroids) {
      cloud_voxelized->points.push_back(point);
    }

    // Convert from PCL point cloud to ROS message
    sensor_msgs::msg::PointCloud2 output_cloud;
    pcl::toROSMsg(*cloud_voxelized, output_cloud);
    output_cloud.header = input_cloud.header;

    return output_cloud;
  }

  rclcpp::TimerBase::SharedPtr timer;

  bool is_fused;
  std::string unique_lidar_source;
  std::string front_lidar_topic;
  std::string left_lidar_topic;
  std::string right_lidar_topic;
  std::string front_lidar_source_frame;
  std::string left_lidar_source_frame;
  std::string right_lidar_source_frame;

  // Downsampling and height filtering parameters:
  int leaf_;
  float z_axis_min_;
  float z_axis_max_;
  float voxel_leafx;
  float voxel_leafy;
  float voxel_leafz;
  float octree_resolution_;
  float cluster_tolerance_;
  int min_cluster_size_;
  int max_cluster_size_;
  int existence_prob_threshold_;
  float pitch_offset_;

  int min_voxel_points;

  std::string target_frame;
  std::string latest_update = "front";

  sensor_msgs::msg::PointCloud2 transformed_pc_front;
  sensor_msgs::msg::PointCloud2 transformed_pc_left;
  sensor_msgs::msg::PointCloud2 transformed_pc_right;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_single_point_cloud;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_front_point_cloud;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_left_point_cloud;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_right_point_cloud;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_tf_pts;
  rclcpp::Publisher<blackandgold_msgs::msg::ErrorReport>::SharedPtr pub_error_msgs;

  std::shared_ptr<tf2_ros::TransformListener> transform_listener_front{nullptr};
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_left{nullptr};
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_right{nullptr};
  std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};

  std::unique_ptr<tf2_ros::Buffer> tf_buffer;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_front;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_left;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_right; 
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CloudTransformerAlt>());
  rclcpp::shutdown();
  return 0;
}
