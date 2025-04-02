#include "CustomHandler.h"

CustomHandler::CustomHandler(const std::string& topic_pub, const std::string& topic_sub)
    : rclcpp::Node("chrono_subscriber"), ChROSHandler(1),
      m_topic_pub(topic_pub), m_topic_sub(topic_sub), m_ticker(0) 
{
    std::cout << "Initializing CustomHandler with topics: "
              << "Publish: " << m_topic_pub << ", Subscribe: " << m_topic_sub << std::endl;
}

bool CustomHandler::Initialize(std::shared_ptr<ChROSInterface> interface) {
    std::cout << "Creating publisher for topic '" << m_topic_pub << "' ..." << std::endl;
    std::cout << "Creating subscriber for topic '" << m_topic_sub << "' ..." << std::endl;

    // Publisher for PX4 VehicleOdometry messages
    m_publisher = this->create_publisher<px4_msgs::msg::VehicleOdometry>(m_topic_pub, 1000);
    std::cout << "Publisher for " << m_topic_pub << " created." << std::endl;

    // Subscriber for PX4 VehicleOdometry messages
    m_subscriber = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
        m_topic_sub, 1000, std::bind(&CustomHandler::topic_callback, this, _1)
    );
    std::cout << "Subscriber for " << m_topic_sub << " created." << std::endl;
    

    return true;
}

#include <chrono>
#include <iomanip>
#include <sstream>

void CustomHandler::Tick(double time) {
    // Create a VehicleOdometry message instance
    px4_msgs::msg::VehicleOdometry msg;
    msg.timestamp = this->now().nanoseconds() / 1000;  // Convert to microseconds
    msg.timestamp_sample = static_cast<uint64_t>(time * 1e6);  // Use time argument for sample timestamp
    msg.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;  
    msg.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_BODY_FRD;

    msg.position[0] = 1.0;  // x
    msg.position[1] = 2.0;  // y
    msg.position[2] = 3.0;  // z
    msg.q[0] = 1.0;  // w
    msg.q[1] = 0.0;  // x
    msg.q[2] = 0.0;  // y
    msg.q[3] = 0.0;  // z
    msg.velocity[0] = 0.1;  // vx
    msg.velocity[1] = 0.2;  // vy
    msg.velocity[2] = 0.3;  // vz
    msg.angular_velocity[0] = 0.01;  // roll rate
    msg.angular_velocity[1] = 0.02;  // pitch rate
    msg.angular_velocity[2] = 0.03;  // yaw rate
    msg.position_variance[0] = 0.01;
    msg.position_variance[1] = 0.01;
    msg.position_variance[2] = 0.01;
    msg.velocity_variance[0] = 0.1;
    msg.velocity_variance[1] = 0.1;
    msg.velocity_variance[2] = 0.1;

    // Convert timestamp (in microseconds) to time point
    auto timestamp_timepoint = std::chrono::system_clock::from_time_t(msg.timestamp / 1000000);

    // Convert time point to human-readable date/time format
    std::time_t timestamp_t = std::chrono::system_clock::to_time_t(timestamp_timepoint);
    std::stringstream time_stream;
    time_stream << std::put_time(std::localtime(&timestamp_t), "%Y-%m-%d %H:%M:%S");
    std::string formatted_time = time_stream.str();

    std::cout << "Publishing VehicleOdometry message with timestamp: " << formatted_time << std::endl;

    // Publish the message
    m_publisher->publish(msg);

    m_ticker++;
}

void CustomHandler::topic_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) const {
    RCLCPP_INFO(this->get_logger(), "Received odometry data: x = %.2f, y = %.2f, z = %.2f", 
                msg->position[0], msg->position[1], msg->position[2]);
}
