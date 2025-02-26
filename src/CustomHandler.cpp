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

void CustomHandler::Tick(double time){
    std::cout << "Publishing " << m_ticker << " ..." << std::endl;
    std_msgs::msg::String msg;
    msg.data = "Custom Handler with CHRONO";
    m_publisher->publish(msg);
    m_ticker++;
}

void CustomHandler::topic_callback(const std_msgs::msg::String::SharedPtr msg) const
{
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
}