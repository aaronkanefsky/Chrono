#include "CustomHandler.h"

CustomHandler::CustomHandler(const std::string& topic_pub, const std::string& topic_sub)
: ChROSHandler(1), m_topic_pub(topic_pub), m_topic_sub(topic_sub), m_ticker(0), Node("Chrono_Subscriber") 
{
}

bool CustomHandler::Initialize(std::shared_ptr<ChROSInterface> interface){
    std::cout << "Creating publisher for topic '" << m_topic_pub  << "' ..." << std::endl;
    std::cout << "Creating subscriber for topic '" << m_topic_sub << "' ..." << std::endl;
    m_publisher = interface->GetNode()->create_publisher<std_msgs::msg::String>(m_topic_pub, 1);
    m_subscriber = interface->GetNode()->create_subscription<std_msgs::msg::String>(m_topic_sub, 10, std::bind(&CustomHandler::topic_callback, this, _1));

    return true;
}

void CustomHandler::Tick(double time) {
    std::cout << "Publishing..." << m_publisher << std::endl;

    // Create custom data instance
    MyData custom_data(42, 3.14, "Custom Data Example");

    // Serialize custom data to a string (this can be changed as per your needs)
    std::string data_string = custom_data.toString();

    // Publish the serialized data as a string message
    std_msgs::msg::String msg;
    msg.data = data_string;
    m_publisher->publish(msg);

    // If you want to remove ticker-based limit, don't use m_ticker or just increment without restricting it
    m_ticker++;
}


void CustomHandler::topic_callback(const std_msgs::msg::String::SharedPtr msg) const
{
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
}