// PROJECT CHRONO - http://projectchrono.org
//
// Original example file used for this project made by Aaron Young
// Current file being edited by Aaron Kanefsky

#include "CustomHandler.h"
// =============================================================================

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    
    // Create the system
    ChSystemNSC sys;
    
    // Create ROS manager
    auto ros_manager = chrono_types::make_shared<ChROSManager>();

    // Create a custom handler
    auto custom_handler = chrono_types::make_shared<CustomHandler>("default", "default");
    ros_manager->RegisterHandler(custom_handler);

    // Finally, initialize the ros manager
    ros_manager->Initialize();

    // ------------

    // Simulation
    double time = 0;
    constexpr double step_size = 2e-3;


    // Simulation loop
    ChRealtimeStepTimer realtime_timer;
    while (1) {
        time = sys.GetChTime();

        // Updates
        if (!ros_manager->Update(time, step_size))
            break;

        sys.DoStepDynamics(step_size);

        realtime_timer.Spin(step_size);
    }

    // Close the Listener
    rclcpp::shutdown();
    return 0;
}
