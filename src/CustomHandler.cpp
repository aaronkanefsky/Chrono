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

    // Publisher for the custom CombinedMessage
    m_publisher = this->create_publisher<px4_msgs::msg::VehicleOdometry>(m_topic_pub, 1000);
    std::cout << "Publisher for " << m_topic_pub << " created." << std::endl;

    // Subscriber for PX4 VehicleOdometry messages (optional, if you still want to receive them)
    m_subscriber = this->create_subscription<px4_msgs::msg::ActuatorMotors>(
        m_topic_sub, 1000, std::bind(&CustomHandler::receiveTopic, this, _1)
    );
    std::cout << "Subscriber for " << m_topic_sub << " created." << std::endl;

    return true;
}


#include <chrono>
#include <iomanip>
#include <sstream>

void CustomHandler::Tick(double time) {
    // Fill the CombinedMessage with data from the PX4 VehicleOdometry message
    px4_msgs::msg::VehicleOdometry msg;
    msg.timestamp = this->now().nanoseconds() / 1000;  // Convert to microseconds
    msg.timestamp_sample = static_cast<uint64_t>(time * 1e6);  // Use time argument for sample timestamp
    msg.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;  
    msg.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_BODY_FRD;
    

    tempTime += 0.01;
    msg.position[0] = 1.0 + tempTime;  // x
    msg.position[1] = 2.0 + tempTime;  // y
    msg.position[2] = 3.0 + tempTime;  // z
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


    // Print the message for debugging (optional)
    std::cout << "Publishing Vehicle Odometry message with timestamp: " << msg.timestamp << std::endl;

    // Publish the message
    m_publisher->publish(msg);

    m_ticker++;
}


void CustomHandler::receiveTopic(const px4_msgs::msg::ActuatorMotors::SharedPtr msg) const {
    RCLCPP_INFO(this->get_logger(), "Received Actuator Motor data at time %d with data [%lu, %f, %f, %f, %f, %f, %f, %f]", msg->timestamp, msg->control[0], msg->control[1], msg->control[2], msg->control[3], msg->control[4], msg->control[5], msg->control[6], msg->control[7]);
}