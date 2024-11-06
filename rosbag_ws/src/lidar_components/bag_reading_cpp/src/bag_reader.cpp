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

        // Publishers for each topic with Reliable QoS
        publisher_front_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/perception/luminar_front/points", qos_profile);
        publisher_right_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/perception/luminar_right/points", qos_profile);
        publisher_left_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/perception/luminar_left/points", qos_profile);

        // Set up the reader and open the bag file
        rosbag2_storage::StorageOptions storage_options;
        storage_options.uri = bag_filename_;
        reader_ = rosbag2_transport::ReaderWriterFactory::make_reader(storage_options);
        reader_->open(storage_options);

        if (reader_->has_next()) {
            auto first_msg = reader_->read_next();
            first_msg_time_ = rclcpp::Time(first_msg->time_stamp);
            current_msg_time_ = first_msg_time_;
            start_time_ = this->now();

            // Deserialize and publish the first message
            publish_message(first_msg);
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
            auto next_msg_time = rclcpp::Time(next_msg->time_stamp);

            // Calculate delay duration without direct time subtraction
            auto delay_duration = std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::nanoseconds(next_msg_time.nanoseconds() - current_msg_time_.nanoseconds()));

            // Schedule the next callback with computed delay
            timer_ = this->create_wall_timer(
                delay_duration, [this, next_msg]() {
                    publish_message(next_msg);
                    schedule_next_message();
                });

            current_msg_time_ = next_msg_time;
        } else if (loop_) {
            // Loop back to the beginning of the bag
            reader_->close();
            rosbag2_storage::StorageOptions storage_options;
            storage_options.uri = bag_filename_;
            reader_->open(storage_options);

            if (reader_->has_next()) {
                auto first_msg = reader_->read_next();
                first_msg_time_ = rclcpp::Time(first_msg->time_stamp);
                current_msg_time_ = first_msg_time_;
                start_time_ = this->now();
                message_count_ = 0;

                publish_message(first_msg);
                schedule_next_message();
            }
        }
    }

    void publish_message(rosbag2_storage::SerializedBagMessageSharedPtr msg)
    {
        rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
        auto ros_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();

        try {
            // Deserialize the message
            serialization_.deserialize_message(&serialized_msg, ros_msg.get());
        } catch (const std::exception & e) {
            RCLCPP_DEBUG(this->get_logger(), "Failed to deserialize message!");
            // RCLCPP_ERROR(this->get_logger(), "Failed to deserialize message: %s", e.what());
            return; // Skip this iteration if deserialization fails
        }
        // serialization_.deserialize_message(&serialized_msg, ros_msg.get());

        if (msg->topic_name == "/perception/luminar_front/points") {
            publisher_front_->publish(*ros_msg);
        } else if (msg->topic_name == "/perception/luminar_right/points") {
            publisher_right_->publish(*ros_msg);
        } else if (msg->topic_name == "/perception/luminar_left/points") {
            publisher_left_->publish(*ros_msg);
        }

        message_count_++;
        RCLCPP_DEBUG(this->get_logger(), "Published message on topic: %s", msg->topic_name.c_str());

        auto elapsed_time = (this->now() - start_time_).seconds();
        double frequency = message_count_ / elapsed_time;
        RCLCPP_DEBUG(this->get_logger(), "Publish frequency: %.2f Hz", frequency);
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_front_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_right_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_left_;
    rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serialization_;
    std::unique_ptr<rosbag2_cpp::Reader> reader_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool loop_;
    std::string bag_filename_;
    int message_count_;
    rclcpp::Time start_time_;
    rclcpp::Time first_msg_time_;
    rclcpp::Time current_msg_time_;
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
