# Chrono-ROS 2 Integration: VehicleOdometry Publisher

## Overview

This project integrates the Project Chrono simulation engine with ROS 2 to publish synthetic PX4 `VehicleOdometry` data based on simulation state and listen to actuator motor commands from PX4.

## Key Components

- `CustomHandler`: A ROS 2 node that extends `ChROSHandler`.
- Publishes PX4 `VehicleOdometry` messages at every simulation tick.
- Subscribes to PX4 `ActuatorMotors` messages.

## CustomHandler Class

`CustomHandler` inherits from both `ChROSHandler` and `rclcpp::Node`. It sets up a publisher and subscriber and connects simulation data to ROS 2 messages.

### Header

```cpp
class CustomHandler : public ChROSHandler, public rclcpp::Node {
public:
    CustomHandler(const std::string& topic_pub, const std::string& topic_sub);
    virtual bool Initialize(std::shared_ptr<ChROSInterface> interface) override;
    virtual void Tick(double time) override;

private:
    rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr m_publisher;
    rclcpp::Subscription<px4_msgs::msg::ActuatorMotors>::SharedPtr m_subscriber;

    void receiveTopic(px4_msgs::msg::ActuatorMotors::SharedPtr msg) const;
};
```
## Adjusting the CMakeLists.txt File
One major issue is the path of the chrono library and px4 messages was made to be a local path. The following lines **MUST** change for the messages to be recognized and compiled:

Line 10: `set(Chrono_DIR "/home/aaron/Documents/acsl-chrono-simulator/libraries/chrono-build/cmake")`

Line 16: `set(CMAKE_PREFIX_PATH "/home/aaron/px4_msgs-build/install/px4_msgs/share:$ENV{CMAKE_PREFIX_PATH}")`

Line 17: `set(px4_msgs_DIR "/home/aaron/px4_msgs-build/install/px4_msgs/share/px4_msgs/cmake")`

Line 40: `/home/aaron/Documents/acsl-chrono-simulator/libraries/chrono/src`

Line 69: `/home/aaron/Documents/acsl-chrono-simulator/libraries/chrono-build/lib/libChronoEngine.so`

Line 70: `/home/aaron/Documents/acsl-chrono-simulator/libraries/chrono-build/lib/libChronoEngine_ros.so`

## Simulation and Testing
You must first source the Ros2 project using the following command (assuming proper installation):

`source /opt/ros/galactic/setup.bash`

Now, to run the executable, go to the `/build/` folder and run `./customhandler`