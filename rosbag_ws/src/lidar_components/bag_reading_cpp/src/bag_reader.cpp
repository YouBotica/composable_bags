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

class PlaybackNode : public rclcpp::Node
{
public:
    PlaybackNode(const std::string & bag_filename, bool loop)
    : Node("play_bag_node"), loop_(loop), bag_filename_(bag_filename), message_count_(0)
    {
        
        rclcpp::QoS qos_profile(rclcpp::KeepLast(10));  // Set depth to 10
        qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);

        // auto qos_profile = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
        // qos_profile.depth(10); // Adjust as needed
        // qos_profile.best_effort();

        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/perception/luminar_front/points", qos_profile); // qos

        // Set up the reader and open the bag file
        rosbag2_storage::StorageOptions storage_options;
        storage_options.uri = bag_filename_;
        reader_ = rosbag2_transport::ReaderWriterFactory::make_reader(storage_options);
        reader_->open(storage_options);

        if (reader_->has_next()) {
            auto first_msg = reader_->read_next();
            first_msg_time_ = rclcpp::Time(first_msg->time_stamp);
            current_msg_time_ = first_msg_time_; // Initialize the current message time

            // Deserialize and publish the first message
            rclcpp::SerializedMessage serialized_msg(*first_msg->serialized_data);
            auto ros_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
            serialization_.deserialize_message(&serialized_msg, ros_msg.get());
            publisher_->publish(*ros_msg);
            message_count_++;
            start_time_ = this->now();

            // Set timer based on the next message's timestamp
            schedule_next_message();
        } else {
            RCLCPP_ERROR(this->get_logger(), "Bag file is empty!");
        }
    }

private:
    void schedule_next_message()
    {
        if (reader_->has_next()) {
            auto next_msg = reader_->read_next();

            // Calculate delay until the next message should be published
            auto next_msg_time = rclcpp::Time(next_msg->time_stamp);
            auto delay = std::chrono::nanoseconds((next_msg_time - current_msg_time_).nanoseconds());

            // Print delay in seconds
            RCLCPP_INFO(this->get_logger(), "Delay: %.2f s", delay.count() / 1e9);

            // Schedule the next callback
            timer_ = this->create_wall_timer(
                delay, [this, next_msg]() {
                    publish_message(next_msg);
                    schedule_next_message();
                });

            current_msg_time_ = next_msg_time; // Update current message time
        } else if (loop_) {
            // Loop back to the beginning of the bag
            reader_->close();
            rosbag2_storage::StorageOptions storage_options;
            storage_options.uri = bag_filename_;
            reader_->open(storage_options);

            // Reset the reference times for looping
            if (reader_->has_next()) {
                auto first_msg = reader_->read_next();
                first_msg_time_ = rclcpp::Time(first_msg->time_stamp);
                current_msg_time_ = first_msg_time_;
                start_time_ = this->now();
                message_count_ = 0;
                
                // Publish the first message again
                publish_message(first_msg);
                schedule_next_message();
            }
        }
    }

    void publish_message(rosbag2_storage::SerializedBagMessageSharedPtr msg)
    {
        if (msg->topic_name == "/perception/luminar_front/points") {
            rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
            auto ros_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
            serialization_.deserialize_message(&serialized_msg, ros_msg.get());

            publisher_->publish(*ros_msg);
            message_count_++;
            RCLCPP_DEBUG(this->get_logger(), "Published message");

            // Log playback frequency
            auto elapsed_time = this->now() - start_time_;
            double frequency = message_count_ / elapsed_time.seconds();
            RCLCPP_INFO(this->get_logger(), "Publish frequency: %.2f Hz", frequency);
        }
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serialization_;
    std::unique_ptr<rosbag2_cpp::Reader> reader_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool loop_;
    std::string bag_filename_;
    int message_count_;
    rclcpp::Time start_time_;
    rclcpp::Time first_msg_time_;
    rclcpp::Time current_msg_time_; // Track the current message time for delays
};

int main(int argc, char ** argv)
{
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <bag> [-l]" << std::endl;
        return 1;
    }

    std::string bag_filename = argv[1];
    bool loop = (argc == 3 && std::string(argv[2]) == "-l");

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PlaybackNode>(bag_filename, loop));
    rclcpp::shutdown();

    return 0;
}
