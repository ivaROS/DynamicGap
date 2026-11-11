#include <gtest/gtest.h>
#include <ros/ros.h>

#include "fake_ros_master.h"

// Shared entry point for the dynamic_gap unit tests. Planner owns a ros::NodeHandle,
// and roscpp blocks on the master when the first NodeHandle is created, so an
// in-process stub master is started and roscpp is pointed at it. No roscore is used.
int main(int argc, char ** argv)
{
    testing::InitGoogleTest(&argc, argv);

    dynamic_gap::FakeRosMaster master;
    if (!master.start())
    {
        std::cerr << "could not start in-process ROS master stub" << std::endl;
        return 1;
    }

    ros::M_string remappings;
    remappings["__master"] = master.uri();
    ros::init(remappings, "dynamic_gap_test",
              ros::init_options::AnonymousName | ros::init_options::NoRosout | ros::init_options::NoSimTime);

    int result = RUN_ALL_TESTS();

    ros::shutdown();
    master.stop();
    return result;
}
