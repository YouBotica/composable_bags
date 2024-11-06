#include <chrono>
#include <string>

// PCL
#include <pcl/filters/crop_box.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl_ros/transforms.hpp>

// Eigen
#include <eigen3/Eigen/Geometry>

// ROS 2
#include <rclcpp/clock.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/rclcpp.hpp>

// Message types
#include "blackandgold_msgs/msg/error_report.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

// TF2
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace tf2 {
inline geometry_msgs::msg::Vector3 toMsg(const Vector3& in) {
  geometry_msgs::msg::Vector3 out;
  out.x = in.getX();
  out.y = in.getY();
  out.z = in.getZ();
  return out;
}

inline void fromMsg(const geometry_msgs::msg::Vector3& in, Vector3& out) {
  out = tf2::Vector3(in.x, in.y, in.z);
}

inline void fromMsg(const geometry_msgs::msg::Point& in, Vector3& out) {
  out = tf2::Vector3(in.x, in.y, in.z);
}
}  // namespace tf2
#endif

using namespace std::chrono_literals;

class CloudTransformer : public rclcpp::Node {
 public:
  CloudTransformer() : Node("cloud_transform") {
    // target frame of the fused and processed point clouds
    perception_target_frame =
        this->declare_parameter<std::string>("perception_reference_frame", "center_of_gravity");
    perception_target_frame = this->get_parameter("perception_reference_frame").as_string();
    
    // FIXME: to be set by parameter. By default, it would work even without subscribing to an
    // odometry source. Should be empty by default.
    odom_topic = this->declare_parameter<std::string>("odom_topic", "/odometry/global_filtered");
    odom_child_frame = this->declare_parameter<std::string>("odom_child_frame", "");
    this->declare_parameter("already_fused", true); // Whether the driver already outputs the ...
    //... pointcloud fused or not (Useful for testing with other team's rosbags)


    // initialize a QOS object for pubs and subs
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 1));
    qos.best_effort();

    // TODO: add more publishers for different processing stages
    pub_tf_pts = this->create_publisher<sensor_msgs::msg::PointCloud2>("/points_filtered",
                                                                       10);  //, rclcpp::QoS(10));
    pub_error_msgs = this->create_publisher<blackandgold_msgs::msg::ErrorReport>(
        "/perception/errors", 10);  //, rclcpp::QoS(10));

    tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    transform_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    cropBox.setNegative(true);
    // set box size and rotation -- this is in peception target frame.
    // FIXME: make these parameters
    // cropBox.setMin(Eigen::Vector4f(-2.5, -1.2, -0.3, 1.0));
    cropBox.setMin(Eigen::Vector4f(-5.0, -1.2, -0.3, 1.0));
    cropBox.setMax(Eigen::Vector4f(3.0, 1.2, 0.5, 1.0));

    bool isFused = this->get_parameter("already_fused").as_bool();
    RCLCPP_INFO(this->get_logger(), isFused ? "True": "False");
    
    if (!isFused) {
      pc_front = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
      pc_left = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
      pc_right = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
      receivePointCloudToLocal(this, sub_front_point_cloud, "/luminar_lidar_front/points",
                             *pc_front.get(), header_front, qos);
      receivePointCloudToLocal(this, sub_left_point_cloud, "/luminar_lidar_left/points",
                              *pc_left.get(), header_left, qos);
      receivePointCloudToLocal(this, sub_right_point_cloud, "/luminar_lidar_right/points",
                              *pc_right.get(), header_right, qos);
      sub_odom_compensator = this->create_subscription<nav_msgs::msg::Odometry>(
          odom_topic, rclcpp::SensorDataQoS(),
          [this](const nav_msgs::msg::Odometry::UniquePtr odom) { this->last_odom = *odom; });
      // periodic tasks
      timer = this->create_wall_timer(33333us, std::bind(&CloudTransformer::timer_callback, this)); 
    }
    else {
      pc_points = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
      // Create a QoS object with the desired settings
      auto qos1 = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

      subscription_fused_pcl = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/luminar_points", qos1,
      [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
          this->only_transform(msg, perception_target_frame);
      });

      //receivePointCloudToLocal(this, sub_fused_pointcloud, "/luminar_points",
      //              *pc_points.get(), header_fused_pcl, qos1);

    }

    // segmentation
    pub_ground = create_publisher<sensor_msgs::msg::PointCloud2>("/points_ground",
                                                                 rclcpp::SystemDefaultsQoS());
    pub_nonground = create_publisher<sensor_msgs::msg::PointCloud2>("/points_nonground",
                                                                    rclcpp::SystemDefaultsQoS());
  }

 private:

  void only_transform(sensor_msgs::msg::PointCloud2::SharedPtr pcl_msg, std::string target_frame) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr local_value;
    std_msgs::msg::Header local_header;

    if (!pcl_msg->data.size()) {
      this->publishLidarError(pcl_msg->header.frame_id);
      return;
    }

    //local_value.empty();
    local_header = pcl_msg->header; // Stores pointcloud2 msg header for future usage
    pcl::moveFromROSMsg(*pcl_msg, *local_value); // Converts from ros2 to pcl msg.
    RCLCPP_INFO(this->get_logger(), "I heard: '%i'", local_value->size()); // OK

    try{
      tf_fused_pcl = tf_buffer->lookupTransform(target_frame, local_header.frame_id,
        tf2::TimePointZero);
    }
    catch (const std::exception& e){
      RCLCPP_INFO(this->get_logger(), "Got exception: '%s'", e.what());
      return;
    }

    init = true;
    RCLCPP_DEBUG(this->get_logger(), "Transform: '%f'", tf_fused_pcl.transform.translation.x); // OK

    
    pcl::PointCloud<pcl::PointXYZI> transformedPointCloud;
    pcl::PointCloud<pcl::PointXYZI> filteredPointCloud;
    sensor_msgs::msg::PointCloud2 transformed_pcl;
    transformed_pcl.header.frame_id = pcl_msg->header.frame_id;
    transformed_pcl.header.stamp = pcl_msg->header.stamp;

    pcl::PointCloud<pcl::PointXYZI> pcSelfRemoved;
    Eigen::Affine3f t(Eigen::Affine3f::Identity());
    t.translate(Eigen::Vector3f(tf_fused_pcl.transform.translation.x, tf_fused_pcl.transform.translation.y,
                                tf_fused_pcl.transform.translation.z));
    t.rotate(Eigen::Quaternionf(tf_fused_pcl.transform.rotation.w, tf_fused_pcl.transform.rotation.x,
                                tf_fused_pcl.transform.rotation.y, tf_fused_pcl.transform.rotation.z));
    cropBox.setTransform(t);
    cropBox.setInputCloud(local_value);
    cropBox.filter(filteredPointCloud);

    //pcl_ros::transformPointCloud(target_frame, tf_fused_pcl, *pcl_msg, transformed_pcl);
    pcl_ros::transformPointCloud(filteredPointCloud, transformedPointCloud, tf_fused_pcl);
    //pcl_ros::transformPointCloud("center_of_gravity", tf_fused_pcl, *pcl_msg, transformed_pcl);

    RCLCPP_DEBUG(this->get_logger(), "Transformed pcl: '%i'", transformedPointCloud.size()); 
    pub_tf_pts->publish(transformed_pcl);


    //RCLCPP_INFO(this->get_logger(), "Got exception: '%s' when transforming pointcloud", e.what());

  }

  tf2::TimePoint tfTimePointFromStamp(const builtin_interfaces::msg::Time& in) {
    return tf2::timeFromSec(static_cast<double>(in.sec) + 1.0e-9 * static_cast<double>(in.nanosec));
  }

  geometry_msgs::msg::TransformStamped twistDisplacement(
      const geometry_msgs::msg::Twist& twist,
      const geometry_msgs::msg::TransformStamped& twist_frame_trans, const rclcpp::Time& tStart,
      const rclcpp::Time& tEnd) {
    geometry_msgs::msg::TransformStamped ret;
    double dt = static_cast<double>(tEnd.nanoseconds() - tStart.nanoseconds()) * 1.e-9;
    ret.header.frame_id = twist_frame_trans.header.frame_id;
    ret.header.stamp = tEnd;
    ret.child_frame_id = ret.header.frame_id;
    // rotate to perception frame first
    tf2::Quaternion q;
    tf2::fromMsg(twist_frame_trans.transform.rotation, q); // From ros quaternion to tf2::Quaternion
    tf2::Vector3 w(twist.angular.x, twist.angular.y, twist.angular.z);
    w = tf2::quatRotate(q, w);  // Rotates vector w by the rotation given by q
    tf2::Vector3 v(twist.linear.x, twist.linear.y, twist.linear.z);
    v = tf2::quatRotate(q, v); // Rotates vector v by the rotation given by q
    tf2::Vector3 x;
    tf2::fromMsg(twist_frame_trans.transform.translation, x);
    v += w.cross(-x);
    // w: is the angular rate vector
    // v: is the linear velocity vector
    ret.transform.translation.x = -v.x() * dt;
    ret.transform.translation.y = -v.y() * dt;
    ret.transform.translation.z = -v.z() * dt;
    tf2::Vector3 dr(-w * dt);
    tf2::Quaternion qr;
    qr.setEuler(dr.z(), dr.y(), dr.x());
    ret.transform.rotation.w = qr.w();
    ret.transform.rotation.x = qr.x();
    ret.transform.rotation.y = qr.y();
    ret.transform.rotation.z = qr.z();
    return ret;
  }

  template <typename T>
  sensor_msgs::msg::PointCloud2 pclToROSMsg(const pcl::PointCloud<T>& cloud,
                                            const std::string& frame_id = "map") {
    sensor_msgs::msg::PointCloud2 cloud_ROS;
    pcl::toROSMsg(cloud, cloud_ROS);
    cloud_ROS.header.frame_id = frame_id;
    return cloud_ROS;
  }

  void timer_callback() {
    //RCLCPP_INFO(this->get_logger(), ("Target: " + perception_target_frame + " From: " + header_fused_pcl.frame_id).c_str());
    if (!init) {
      try {
        if (!isFused){//(!this->get_parameter("already_fused").as_bool()){
          // initialize tf at intial startup.
          // Required for stitching the clouds considering unsynchronized lidar data when clouds are unfused
          tf_front = tf_buffer->lookupTransform(perception_target_frame, header_front.frame_id,
                                                tf2::TimePointZero);
          tf_left = tf_buffer->lookupTransform(perception_target_frame, header_left.frame_id,
                                              tf2::TimePointZero);
          tf_right = tf_buffer->lookupTransform(perception_target_frame, header_right.frame_id,
                                                tf2::TimePointZero);
          if (odom_child_frame.length()) last_odom.child_frame_id = odom_child_frame;
          tf_odom_child = tf_buffer->lookupTransform(perception_target_frame,
                                                    last_odom.child_frame_id, tf2::TimePointZero);
          init = true;
        }

        else {
          RCLCPP_INFO(this->get_logger(), (" From: " + header_fused_pcl.frame_id).c_str());
          tf_fused_pcl = tf_buffer->lookupTransform(perception_target_frame, this->header_fused_pcl.frame_id,
                                      tf2::TimePointZero);
          if (odom_child_frame.length()) last_odom.child_frame_id = odom_child_frame;
          tf_odom_child = tf_buffer->lookupTransform(perception_target_frame,
                                                    last_odom.child_frame_id, tf2::TimePointZero);
          
          init = true;
        }
      }
        catch (const std::exception& e) {
        // tf is not ready yet.
        RCLCPP_INFO_THROTTLE(this->get_logger(), *(this->get_clock()), 1000,
                             "Cloud transform still waiting to initialize... '%s'", e.what());
        return;
      }
    }

    // Common variables for both cases (fused | unfused)
    pcl::PointCloud<pcl::PointXYZI> pc_fused; 
    rclcpp::Time outputStamp;

    // Figure out this part of the code
    if (!isFused){
      // Check which one is the latest pointcloud received
      rclcpp::Time tFront = header_front.stamp; 
      outputStamp = tFront;
      double tOut = outputStamp.seconds();
      rclcpp::Time tLeft = rclcpp::Time(header_left.stamp);
      if (double t = tLeft.seconds() > tOut) {
        outputStamp = tLeft;
        tOut = t;
      }
      rclcpp::Time tRight = rclcpp::Time(header_right.stamp);
      if (tRight.seconds() > tOut) outputStamp = tRight;

      geometry_msgs::msg::TransformStamped tf_front_sync, tf_left_sync, tf_right_sync;
      tf2::doTransform(tf_left, tf_left_sync,
                  twistDisplacement(last_odom.twist.twist, tf_odom_child, tLeft, outputStamp));
      tf2::doTransform(tf_right, tf_right_sync,
                  twistDisplacement(last_odom.twist.twist, tf_odom_child, tRight, outputStamp)); 
      
      // Step 1: stitch together the point clouds
      // pcl_ros::transformPointCloud(perception_target_frame, tf_front_sync, pc_front, pc_front);
      pcl::PointCloud<pcl::PointXYZI> pc_rect_f;
      removeSelfBoxAndSyncTransform(tf_front, tFront, outputStamp, pc_front, pc_rect_f);

      pcl::PointCloud<pcl::PointXYZI> pc_rect_l;
      removeSelfBoxAndSyncTransform(tf_left, tLeft, outputStamp, pc_left, pc_rect_l);

      pcl::PointCloud<pcl::PointXYZI> pc_rect_r;
      removeSelfBoxAndSyncTransform(tf_right, tRight, outputStamp, pc_right, pc_rect_r);

      pcl::PointCloud<pcl::PointXYZI> pc_fused = pc_rect_f + pc_rect_l + pc_rect_r;     
    }
    else {
      rclcpp::Time tFused = header_front.stamp;
      geometry_msgs::msg::TransformStamped tf_fused_sync;
      outputStamp = tFused;
      //double tOut = outputStamp.seconds();
      tf2::doTransform(tf_fused_pcl, tf_fused_sync,
            twistDisplacement(last_odom.twist.twist, tf_odom_child, tFused, outputStamp));
      
      removeSelfBoxAndSyncTransform(tf_fused_pcl, tFused, outputStamp, pc_points, pc_fused);
      
    }

    pc_fused.header.frame_id = perception_target_frame;
    pc_fused.header.stamp = outputStamp.nanoseconds() / 1e3;

    auto result_pc_msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(pc_fused, *result_pc_msg.get());

    pub_tf_pts->publish(std::move(result_pc_msg));

    // Step 2: segment the point cloud between ground and non-ground
    pcl::PointCloud<pcl::PointXYZI> pc_ground;
    pcl::PointCloud<pcl::PointXYZI> pc_non_ground;
    double time_taken;

    pub_ground->publish(pclToROSMsg(pc_ground, perception_target_frame));
    pub_nonground->publish(pclToROSMsg(pc_non_ground, perception_target_frame));
  }

  // do not transform and fuse yet... as when the lidars are not time-synchronized, the target
  // frames may have moved over the time difference. store a local copy in the callback.
  // one-liner template that subscribes to a topic and stores the variable locally to a class.
  template <class NodeT>
  void receivePointCloudToLocal(NodeT* this_ptr,
                                typename std::shared_ptr<rclcpp::SubscriptionBase>& subscriber,
                                const std::string& topic_name,
                                pcl::PointCloud<pcl::PointXYZI>& local_value,
                                std_msgs::msg::Header& local_header,
                                const rclcpp::QoS& qos = rclcpp::SensorDataQoS()) {
    subscriber =
        static_cast<rclcpp::Node*>(this_ptr)->create_subscription<sensor_msgs::msg::PointCloud2>(
            topic_name, qos,
            [&this_ptr, &local_value,
             &local_header](const sensor_msgs::msg::PointCloud2::UniquePtr pcl_msg) {
              // [&this_ptr, &local_value, &topic_name](const
              // sensor_msgs::msg::PointCloud2::UniquePtr pcl_msg){
              // RCLCPP_INFO(this_ptr->get_logger(), (topic_name + " received").c_str());
              if (!pcl_msg->data.size()) {
                this_ptr->publishLidarError(pcl_msg->header.frame_id);
                return;
              }
              local_header = pcl_msg->header; // Stores pointcloud2 msg header for future usage
              RCLCPP_INFO(this_ptr->get_logger(), (" From: " + local_header.frame_id).c_str());
              pcl::moveFromROSMsg(*pcl_msg, local_value); // Converts from ros2 to pcl msg.
            });
  }

  void removeSelfBoxAndSyncTransform(const geometry_msgs::msg::TransformStamped& staticTf,
                                     const rclcpp::Time& srcTime, const rclcpp::Time& syncTime,
                                     const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloudIn,
                                     pcl::PointCloud<pcl::PointXYZI>& cloudOut) {
    geometry_msgs::msg::TransformStamped tfSync;
    tf2::doTransform(staticTf, tfSync,
                     twistDisplacement(last_odom.twist.twist, tf_odom_child, srcTime, syncTime));
    pcl::PointCloud<pcl::PointXYZI> pcSelfRemoved;
    Eigen::Affine3f t(Eigen::Affine3f::Identity());
    t.translate(Eigen::Vector3f(staticTf.transform.translation.x, staticTf.transform.translation.y,
                                staticTf.transform.translation.z));
    t.rotate(Eigen::Quaternionf(staticTf.transform.rotation.w, staticTf.transform.rotation.x,
                                staticTf.transform.rotation.y, staticTf.transform.rotation.z));
    cropBox.setTransform(t);
    cropBox.setInputCloud(cloudIn);
    cropBox.filter(pcSelfRemoved);
    pcl_ros::transformPointCloud(pcSelfRemoved, cloudOut, tfSync);
  }

  void publishLidarError(const std::string& which) {
    auto error_msg = std::make_unique<blackandgold_msgs::msg::ErrorReport>();
    error_msg->description = "No pointcloud received from lidar " + which;
    error_msg->severity = 3;  // No lidar data, the vehicle is blind
    error_msg->module = "Perception";
    error_msg->origin = "cloud_transform";
    error_msg->lifetime = 1;
    pub_error_msgs->publish(std::move(error_msg));
  }

  rclcpp::TimerBase::SharedPtr timer;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> transform_listener{nullptr};
  std::string perception_target_frame, odom_topic, odom_child_frame;
  bool isFused;
  bool init = false;

  pcl::PointCloud<pcl::PointXYZI>::Ptr pc_front, pc_left, pc_right, pc_points;
  std_msgs::msg::Header header_front, header_left, header_right, header_fused_pcl;
  pcl::CropBox<pcl::PointXYZI> cropBox;
  nav_msgs::msg::Odometry last_odom;
  geometry_msgs::msg::TransformStamped tf_front, tf_left, tf_right, tf_fused_pcl, tf_odom_child, tf_vel_left,
      tf_vel_front, tf_vel_right;

  rclcpp::SubscriptionBase::SharedPtr sub_front_point_cloud, sub_left_point_cloud,
      sub_right_point_cloud, sub_odom_compensator, sub_fused_pointcloud;
  
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_fused_pcl;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_tf_pts;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_ground, pub_nonground;
  rclcpp::Publisher<blackandgold_msgs::msg::ErrorReport>::SharedPtr pub_error_msgs;


  // FIXME: make the subscription topics parameters

};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CloudTransformer>());
  rclcpp::shutdown();
  return 0;
}
