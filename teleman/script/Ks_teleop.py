#!/usr/bin/env python
import rospy
import tf.transformations
import numpy as np
import dynamic_reconfigure.client

from interactive_markers.interactive_marker_server import InteractiveMarkerServer, InteractiveMarkerFeedback
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl
from geometry_msgs.msg import PoseStamped, WrenchStamped
from franka_msgs.msg import FrankaState

from std_msgs.msg import String

robot_pose = PoseStamped()
pose_pub = None
p_door_x = 0.65

param_node = '/teleman/cartesian_impedance_example_controllerdynamic_reconfigure_compliance_param_node'

# [[min_x, max_x], [min_y, max_y], [min_z, max_z]]
position_limits = [[0.3, 0.9], [-0.1, 0.38], [0.05, 0.9]] # door
def param_callback(config):
    True
    #rospy.loginfo("Config set")

def publisher_callback(msg, link_name):
    robot_pose.header.frame_id = link_name
    robot_pose.header.stamp = rospy.Time(0)
    pose_pub.publish(robot_pose)

def next_trial_callback(data):
    rospy.loginfo("next trial callback")

    robot_pose.pose.position.x = 0.45367541276468626
    robot_pose.pose.position.y = 0.3251109849090883
    robot_pose.pose.position.z = 0.5804182673822611
    pose_pub.publish(robot_pose)

def pose_callback(data):

    if robot_pose.pose.position.x != data.pose.position.x:
        robot_pose.pose.position.x = max([min([data.pose.position.x, position_limits[0][1]]), position_limits[0][0]])
        robot_pose.pose.position.y = max([min([data.pose.position.y, position_limits[1][1]]), position_limits[1][0]])
        robot_pose.pose.position.z = max([min([data.pose.position.z - 0.9, position_limits[2][1]]), position_limits[2][0]])

        # robot_pose.pose.orientation = data.pose.orientation
        #robot_pose.pose.orientation.x = 0.21
        #robot_pose.pose.orientation.y = 0.65
        #robot_pose.pose.orientation.z = 0.29
        #robot_pose.pose.orientation.w = 0.66

        #rospy.loginfo("published position: " + str(robot_pose.pose.position))

    server.applyChanges()

def wait_for_initial_pose():
    msg = rospy.wait_for_message("franka_state_controller/franka_states", FrankaState)  # type: FrankaState

    initial_quaternion = tf.transformations.quaternion_from_matrix(np.transpose(np.reshape(msg.O_T_EE, (4, 4))))
    initial_quaternion = initial_quaternion / \
        np.linalg.norm(initial_quaternion)

    robot_pose.pose.position.x = msg.O_T_EE[12]
    robot_pose.pose.position.y = msg.O_T_EE[13]
    robot_pose.pose.position.z = msg.O_T_EE[14]

    robot_pose.pose.orientation.x = initial_quaternion[0]
    robot_pose.pose.orientation.y = initial_quaternion[1]
    robot_pose.pose.orientation.z = initial_quaternion[2]
    robot_pose.pose.orientation.w = initial_quaternion[3]

    rospy.loginfo("current position: " + str(robot_pose.pose.position))
    rospy.loginfo("current orientation: " + str(robot_pose.pose.orientation))

if __name__ == "__main__":
    rospy.init_node("robot_teleman")
    rospy.loginfo("teleop with Ks starts")
    listener = tf.TransformListener()
    link_name = rospy.get_param("~link_name")
    wait_for_initial_pose()

    client = dynamic_reconfigure.client.Client(param_node, timeout=30, config_callback=param_callback)
    client.update_configuration({'translational_stiffness': 500.0, 'rotational_stiffness': 20.0})

    pose_pub = rospy.Publisher("equilibrium_pose", PoseStamped, queue_size=10)
    pose_sub = rospy.Subscriber("/teleop_franka", PoseStamped, pose_callback)

    next_trial_sub = rospy.Subscriber("/next_trial", String, next_trial_callback)

    server = InteractiveMarkerServer("equilibrium_pose_marker")
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = link_name
    int_marker.scale = 0.3
    int_marker.name = "equilibrium_pose"
    int_marker.pose = robot_pose.pose

    # run pose publisher
    rospy.Timer(rospy.Duration(0.005), lambda msg: publisher_callback(msg, link_name))

    rospy.spin()
