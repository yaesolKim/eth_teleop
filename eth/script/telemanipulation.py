#!/usr/bin/env python
import rospy
import tf.transformations
import numpy as np

from interactive_markers.interactive_marker_server import InteractiveMarkerServer, InteractiveMarkerFeedback
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl
from geometry_msgs.msg import PoseStamped
from franka_msgs.msg import FrankaState

robot_pose = PoseStamped()
pose_pub = None

isStatic = True
new_data = False

# [[min_x, max_x], [min_y, max_y], [min_z, max_z]]
position_limits = [[-0.6, 0.6], [-0.6, 0.6], [0.05, 0.9]]

def publisher_callback(msg, link_name):
    robot_pose.header.frame_id = link_name
    robot_pose.header.stamp = rospy.Time(0)
    pose_pub.publish(robot_pose)
    new_data = False

def robot_callback(data):
    if data.pose.position.x != robot_pose.pose.position.x * (0.1) and str(data.header.frame_id) == 't':
        robot_pose.pose.position.x = data.pose.position.x * (0.1)
        robot_pose.pose.position.y = data.pose.position.y * (-0.1)
        robot_pose.pose.position.z = data.pose.position.z * (0.1)
        robot_pose.pose.orientation = data.pose.orientation

        new_data = True

    server.applyChanges()


def process_feedback(feedback):
    if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
        robot_pose.pose.position.x = max([min([feedback.pose.position.x,
                                          position_limits[0][1]]),
                                          position_limits[0][0]])
        robot_pose.pose.position.y = max([min([feedback.pose.position.y,
                                          position_limits[1][1]]),
                                          position_limits[1][0]])
        robot_pose.pose.position.z = max([min([feedback.pose.position.z,
                                          position_limits[2][1]]),
                                          position_limits[2][0]])
        robot_pose.pose.orientation = feedback.pose.orientation
    server.applyChanges()


def wait_for_initial_pose():
    msg = rospy.wait_for_message("franka_state_controller/franka_states", FrankaState)  # type: FrankaState

    initial_quaternion = tf.transformations.quaternion_from_matrix(np.transpose(np.reshape(msg.O_T_EE, (4, 4))))
    initial_quaternion = initial_quaternion / \
        np.linalg.norm(initial_quaternion)

    robot_pose.pose.orientation.x = initial_quaternion[0]
    robot_pose.pose.orientation.y = initial_quaternion[1]
    robot_pose.pose.orientation.z = initial_quaternion[2]
    robot_pose.pose.orientation.w = initial_quaternion[3]
    robot_pose.pose.position.x = msg.O_T_EE[12]
    robot_pose.pose.position.y = msg.O_T_EE[13]
    robot_pose.pose.position.z = msg.O_T_EE[14]


if __name__ == "__main__":
    rospy.init_node("robot_teleman")
    listener = tf.TransformListener()
    link_name = rospy.get_param("~link_name")

    wait_for_initial_pose()

    pose_pub = rospy.Publisher("equilibrium_pose_teleman", PoseStamped, queue_size=10)
    pose_sub = rospy.Subscriber("/robot", PoseStamped, robot_callback)

    server = InteractiveMarkerServer("equilibrium_pose_teleman_marker")
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = link_name
    int_marker.scale = 0.3
    int_marker.name = "equilibrium_pose_teleman"
    int_marker.pose = robot_pose.pose

    # run pose publisher
    rospy.Timer(rospy.Duration(0.005), lambda msg: publisher_callback(msg, link_name))

    rospy.spin()
