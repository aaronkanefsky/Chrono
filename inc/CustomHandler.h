#ifndef CUSTOM_HANDLER_H
#define CUSTOM_HANDLER_H

// Chrono Libraries
#include "chrono/core/ChTypes.h"

#include "chrono/core/ChRealtimeStep.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"

#include "chrono_ros/ChROSManager.h"
#include "chrono_ros/ChROSHandler.h"
#include "chrono_ros/handlers/ChROSClockHandler.h"
#include "chrono_ros/handlers/ChROSTFHandler.h"
#include "chrono_ros/handlers/ChROSBodyHandler.h"


// Others
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <chrono>   // THIS IS NOT THE ROSCHRONO LIBRARY, THIS IS TIME

using namespace chrono;
using namespace chrono::ros;
using std::placeholders::_1;

class CustomHandler : public ChROSHandler, public rclcpp::Node {
public:
    CustomHandler(const std::string& topic_pub, const std::string& topic_sub);
    virtual bool Initialize(std::shared_ptr<ChROSInterface> interface) override;
    virtual void Tick(double time) override;

private:
    const std::string m_topic_pub, m_topic_sub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_publisher;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_subscriber;
    int m_ticker;

    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const;
};

#endif