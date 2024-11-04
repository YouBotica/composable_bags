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
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/perception/luminar_front/puntos", 10);
        timer_ = this->create_wall_timer(
                35ms, std::bind(&PlaybackNode::timer_callback, this));

        rosbag2_storage::StorageOptions storage_options;
        storage_options.uri = bag_filename_;
        reader_ = rosbag2_transport::ReaderWriterFactory::make_reader(storage_options);
        reader_->open(storage_options);

        start_time_ = this->now();
    }

private:
    void timer_callback()
    {
        if (reader_->has_next()) {
            auto msg = reader_->read_next();

            if (msg->topic_name == "/perception/luminar_front/points") {
                rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
                auto ros_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
                serialization_.deserialize_message(&serialized_msg, ros_msg.get());

                publisher_->publish(*ros_msg);
                message_count_++;
                RCLCPP_DEBUG(this->get_logger(), "Published message");

                auto current_time = this->now();
                auto elapsed_time = current_time - start_time_;
                double frequency = message_count_ / elapsed_time.seconds();
                RCLCPP_DEBUG(this->get_logger(), "Publish frequency: %.2f Hz", frequency);
            }
        } else if (loop_) {
            // Close and reopen the reader to loop playback
            reader_->close();
            rosbag2_storage::StorageOptions storage_options;
            storage_options.uri = bag_filename_;
            reader_->open(storage_options);
            RCLCPP_INFO(this->get_logger(), "Looping bag playback");
        }
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serialization_;
    std::unique_ptr<rosbag2_cpp::Reader> reader_;
    bool loop_;
    std::string bag_filename_;
    int message_count_;
    rclcpp::Time start_time_;
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
