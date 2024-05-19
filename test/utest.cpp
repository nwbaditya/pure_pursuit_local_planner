#include <gtest/gtest.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include "pure_pursuit_local_planner/pure_pursuit_planner.h" // Include the header file for the class you want to test

using namespace pure_pursuit_local_planner;
// Define a fixture for the test
class PurePursuitPlannerROSTest : public ::testing::Test {
protected:
    // Declare any variables needed for the tests
    PurePursuitPlannerROS planner;
    std::vector<geometry_msgs::PoseStamped> global_plan;

    // Set up the test fixture
    void SetUp() override {
        // Initialize any variables needed for the tests
        // Here you can initialize the planner or any other objects
        // global_plan can be initialized with some sample data if needed
    }

    // Tear down the test fixture
    void TearDown() override {
        // Clean up any resources used for the tests
    }
};

// Test the setPlan function
TEST_F(PurePursuitPlannerROSTest, testSetPlan) {
    // Call the setPlan function with some sample global plan
    bool result = planner.setPlan(global_plan);

    // Check if the result is as expected
    EXPECT_TRUE(result); // You can modify this based on the expected behavior
}

// Add more tests for other public functions if needed
// Example:
// TEST_F(PurePursuitPlannerROSTest, testComputeVelocityCommands) {
//     // Call the computeVelocityCommands function with some sample data
//     geometry_msgs::Twist cmd_vel;
//     bool result = planner.computeVelocityCommands(cmd_vel);
//
//     // Check if the result is as expected
//     EXPECT_TRUE(result); // You can modify this based on the expected behavior
// }

int main(int argc, char **argv) {
    // Initialize ROS before running the tests
    ros::init(argc, argv, "pure_pursuit_planner_ros_test");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
