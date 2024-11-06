#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_storage_default_plugins/sqlite/sqlite_storage.hpp>

#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <iomanip>
#include <sstream>

class BagRecorderNode : public rclcpp::Node
{
public:
    BagRecorderNode()
    : Node("bag_recorder_node")
    {
        // Declare parameters for topics and base filename
        this->declare_parameter<std::string>("luminar_front_topic", "/perception/luminar_front/points");
        this->declare_parameter<std::string>("luminar_left_topic", "/perception/luminar_left/points");
        this->declare_parameter<std::string>("luminar_right_topic", "/perception/luminar_right/points");
        this->declare_parameter<std::string>("bag_filename", "/home/pair-andres/Desktop/test_bag_play/rosbag2_lidar");

        // Get topics from parameters
        topics_to_record_.push_back(this->get_parameter("luminar_front_topic").as_string());
        topics_to_record_.push_back(this->get_parameter("luminar_left_topic").as_string());
        topics_to_record_.push_back(this->get_parameter("luminar_right_topic").as_string());

        // Get base bag filename from parameter and append timestamp
        std::string base_filename = this->get_parameter("bag_filename").as_string();
        std::string bag_filename = generateTimestampedFilename(base_filename);

        // Configure storage options for MCAP format
        rosbag2_storage::StorageOptions storage_options;
        storage_options.uri = bag_filename;
        storage_options.storage_id = "mcap";  // Specify .mcap format

        // Configure writer and open bag file
        writer_ = std::make_shared<rosbag2_cpp::Writer>();
        writer_->open(storage_options);

        // Subscribe to the specified topics and set up callback to record messages
        for (const auto& topic : topics_to_record_) {
            auto callback = [this, topic](const std::shared_ptr<rclcpp::SerializedMessage> msg) {
                writer_->write(msg, topic, "sensor_msgs/msg/PointCloud2", this->get_clock()->now());
            };

            auto subscription = this->create_generic_subscription(
                topic,
                "sensor_msgs/msg/PointCloud2",
                rclcpp::QoS(10),
                callback,
                rclcpp::SubscriptionOptions());

            subscriptions_.push_back(subscription);

            RCLCPP_INFO(this->get_logger(), "Recording topic: %s", topic.c_str());
        }

        RCLCPP_INFO(this->get_logger(), "Recording started. Saving to %s", bag_filename.c_str());
    }

    ~BagRecorderNode() override
    {
        // Close the writer when the node is destroyed
        writer_->close();
        // RCLCPP_INFO(this->get_logger(), "Recording finished and saved. at %s", writer_->get_relative_file_path().c_str());
    }

private:
    std::vector<std::string> topics_to_record_;
    std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
    std::shared_ptr<rosbag2_cpp::Writer> writer_;

    // Helper function to generate a filename with a timestamp
    std::string generateTimestampedFilename(const std::string& base_filename)
    {
        // Get current time and format it
        auto now = std::chrono::system_clock::now();
        std::time_t now_c = std::chrono::system_clock::to_time_t(now);
        std::tm now_tm = *std::localtime(&now_c);

        // Format: rosbag2_lidar_YYYY-MM-DD_HHMMSS.mcap
        std::ostringstream oss;
        oss << base_filename << "_"
            << std::put_time(&now_tm, "%Y-%m-%d_%H%M%S"); //<< ".mcap";
        
        return oss.str();
    }
};

int main(int argc, char* argv[]) {

    rclcpp::init(argc, argv);
    auto node = std::make_shared<BagRecorderNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
