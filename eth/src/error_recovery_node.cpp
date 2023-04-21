#include <ros/ros.h>
#include <iostream>
#include <actionlib/server/simple_action_server.h>
#include <franka_msgs/ErrorRecoveryAction.h>
#include <franka_msgs/FrankaState.h>
#include "std_msgs/String.h"

class ErrorSubscribePublishRecover {

public:
    ErrorSubscribePublishRecover() {
        error_recover_pub = node_handle.advertise<franka_msgs::ErrorRecoveryActionGoal>("/franka_control/error_recovery/goal", 1);
        error_sub = node_handle.subscribe("/franka_state_controller/franka_states", 1, &ErrorSubscribePublishRecover::callback, this);
    }

    void callback(const franka_msgs::FrankaState& state) {
        //ROS_INFO("%d", state.robot_mode);
        if(state.robot_mode == 4) {
          ag = {};
          error_recover_pub.publish(ag);
        }
    }

private:
  ros::NodeHandle   node_handle;
  ros::Publisher    error_recover_pub;
  ros::Subscriber   error_sub;
  franka_msgs::ErrorRecoveryActionGoal ag;
};


int main(int argc, char** argv) {
  ros::init(argc, argv, "error_recovery_node");
  ErrorSubscribePublishRecover ErrorRecover;

  ROS_INFO("RECOVER ALL ERRORS!");

  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::waitForShutdown();
}
