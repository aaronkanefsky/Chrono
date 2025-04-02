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

// ROS2 & PX4 Messages
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include <chrono>  // THIS IS NOT THE ROSCHRONO LIBRARY, THIS IS TIME

using namespace chrono;
using namespace chrono::ros;
using std::placeholders::_1;

// MyData class definition
class MyData {
public:
    int field1;
    double field2;
    std::string field3;

    MyData(int f1 = 0, double f2 = 0.0, const std::string& f3 = "")
        : field1(f1), field2(f2), field3(f3) {}

    std::string toString() const {
        return "Field1: " + std::to_string(field1) + 
               ", Field2: " + std::to_string(field2) + 
               ", Field3: " + field3;
    }
};

class CustomHandler : public ChROSHandler, public rclcpp::Node {
public:
    CustomHandler(const std::string& topic_pub = "default_out", 
                  const std::string& topic_sub = "default_in");

    virtual bool Initialize(std::shared_ptr<ChROSInterface> interface) override;
    virtual void Tick(double time) override;

private:
    const std::string m_topic_pub, m_topic_sub;
    rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr m_publisher;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr m_subscriber;
    int m_ticker = 0;
    rclcpp::TimerBase::SharedPtr m_timer; 

    void topic_callback(px4_msgs::msg::VehicleOdometry::SharedPtr msg) const;
    MyData create_custom_data() const;
};

#endif // CUSTOM_HANDLER_H
