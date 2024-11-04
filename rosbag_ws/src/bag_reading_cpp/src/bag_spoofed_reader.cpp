#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rosbag2_transport/reader_writer_factory.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

using namespace std::chrono_literals;

namespace bag_reading_cpp{
    class PlaybackSpoofedNode : public rclcpp::Node
    {
    public:
        PlaybackSpoofedNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
        : Node("play_bag_spoofed_node", options), message_count_(0)
        {
            // Declare parameters and retrieve them
            this->declare_parameter<std::string>("bag_path", "");
            this->declare_parameter<bool>("loop", false);
            this->get_parameter("bag_path", bag_path_);
            this->get_parameter("loop", loop_);

            RCLCPP_INFO(this->get_logger(), "Bag path: %s", bag_path_.c_str());

            if (bag_path_.empty()) {
                RCLCPP_ERROR(this->get_logger(), "No bag file specified.");
                return;
            }

            // Publishers for each topic
            publisher_front_points_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/perception/luminar_front/points", 10);
            publisher_left_points_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/perception/luminar_left/points", 10);
            publisher_right_points_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/perception/luminar_right/points", 10);

            timer_ = this->create_wall_timer(
                35ms, std::bind(&PlaybackSpoofedNode::timer_callback, this));

            // Set up the reader and open the bag file
            rosbag2_storage::StorageOptions storage_options;
            storage_options.uri = bag_path_;
            storage_options.storage_id = "mcap";
            reader_ = rosbag2_transport::ReaderWriterFactory::make_reader(storage_options);
            reader_->open(storage_options);

            start_time_ = this->now();
        }

    private:
        void timer_callback()
        {
            if (reader_->has_next()) {
                auto msg = reader_->read_next();

                rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
                auto ros_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();

                try {
                    // Deserialize the message
                    serialization_.deserialize_message(&serialized_msg, ros_msg.get());
                } catch (const std::exception & e) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to deserialize message!");
                    // RCLCPP_ERROR(this->get_logger(), "Failed to deserialize message: %s", e.what());
                    return; // Skip this iteration if deserialization fails
                }

                auto time_stamp = ros_msg->header.stamp;
                RCLCPP_INFO(this->get_logger(), "Message timestamp: %d.%d", time_stamp.sec, time_stamp.nanosec);

                // Publish the message to the appropriate topic
                if (msg->topic_name == "/perception/luminar_front/points") {
                    publisher_front_points_->publish(*ros_msg);
                } else if (msg->topic_name == "/perception/luminar_left/points") {
                    publisher_left_points_->publish(*ros_msg);
                } else if (msg->topic_name == "/perception/luminar_right/points") {
                    publisher_right_points_->publish(*ros_msg);
                } else {
                    RCLCPP_WARN(this->get_logger(), "Unknown topic: %s", msg->topic_name.c_str());
                    return;
                }

                message_count_++;
                RCLCPP_DEBUG(this->get_logger(), "Published message on topic %s", msg->topic_name.c_str());

                auto current_time = this->now();
                auto elapsed_time = current_time - start_time_;
                double frequency = message_count_ / elapsed_time.seconds();
                RCLCPP_DEBUG(this->get_logger(), "Publish frequency: %.2f Hz", frequency);

            } else if (loop_) {
                // Close and reopen the reader to loop playback
                reader_->close();
                rosbag2_storage::StorageOptions storage_options;
                storage_options.uri = bag_path_;
                storage_options.storage_id = "mcap";
                reader_->open(storage_options);
                RCLCPP_INFO(this->get_logger(), "Looping bag playback");
            }
        }

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_front_points_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_left_points_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_right_points_;
        rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serialization_;
        std::unique_ptr<rosbag2_cpp::Reader> reader_;
        bool loop_;
        std::string bag_path_;
        int message_count_;
        rclcpp::Time start_time_;
    };
} // namespace bag_reading_cpp

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(bag_reading_cpp::PlaybackSpoofedNode)
