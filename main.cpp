#include <thread>  // For thread functionality
#include <atomic>   // For atomic bool
#include "CustomHandler.h"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    
    // Create the Chrono system
    ChSystemNSC sys;
    
    // Create ROS manager
    auto ros_manager = chrono_types::make_shared<ChROSManager>();

    // Create a custom handler (publisher & subscriber)
    auto custom_handler = chrono_types::make_shared<CustomHandler>("/fmu/out/vehicle_odometry", "/fmu/in/actuator_motors");
    ros_manager->RegisterHandler(custom_handler);

    // Initialize the ROS manager
    ros_manager->Initialize();

    // ------------

    // Atomic flag to signal shutdown
    std::atomic<bool> shutdown_flag(false);

    // Create a ROS 2 spin thread for custom_handler (since it's a Node)
    std::thread ros_thread([&]() {
        while (!shutdown_flag.load()) {
            rclcpp::spin_some(custom_handler);  // Handle ROS callbacks aggressively
        }
    });

    // Simulation parameters
    double time = 0;
    constexpr double step_size = 0.1;  // Determines the speed (lower number => faster)
    ChRealtimeStepTimer realtime_timer;

    // Run simulation loop
    while (rclcpp::ok() && !shutdown_flag.load()) {  // Ensure ROS 2 is still running
        time = sys.GetChTime();

        // Update ROS manager
        if (!ros_manager->Update(time, step_size)) break;

        // Run Chrono simulation step
        sys.DoStepDynamics(step_size);

        // Publish messages
        custom_handler->Tick(time);

        // Keep real-time pacing
        realtime_timer.Spin(step_size);
    }

    // Signal the spinning thread to shut down
    shutdown_flag.store(true);

    // Wait for the ROS spinning thread to finish
    ros_thread.join();

    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}
