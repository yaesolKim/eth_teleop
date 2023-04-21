#!/usr/bin/env python
import rospy
import tf.transformations
import numpy as np
import dynamic_reconfigure.client

from interactive_markers.interactive_marker_server import InteractiveMarkerServer, InteractiveMarkerFeedback
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl
from geometry_msgs.msg import PoseStamped, WrenchStamped, Point
from franka_msgs.msg import FrankaState


from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *

robot_pose = PoseStamped()
roughness_level = 0
pose_pub = None
new_data = False
isContact = False
isDynamic_old = False

param_node = '/cartesian_impedance_example_controllerdynamic_reconfigure_compliance_param_node'

ext_F_threshold = 30.0

# [[min_x, max_x], [min_y, max_y], [min_z, max_z]]
position_limits = [[-0.6, 0.6], [-0.6, 0.6], [0.05, 0.9]]

def param_callback(config):
    rospy.loginfo("Config set")

def publisher_callback(msg, link_name):
    robot_pose.header.frame_id = link_name
    robot_pose.header.stamp = rospy.Time(0)
    pose_pub.publish(robot_pose)
    new_data = False

# Set robot contact state based on the external force
def force_callback(data):
    global isContact
    if data.wrench.force.z > ext_F_threshold:
        isContact = True
    else:
        isContact = False

#set contact configuration: contact surface pose(position and orientation), object property(static/dynamic), surface property(level of textures)
def robot_callback(data):
    global client
    global roughness_level
    global isDynamic_old
    global int_marker

    rospy.loginfo("data_pose: " + str(data.pose.position.x) + " " + str(data.pose.position.y) + " " + str(data.pose.position.z))

    robot_pose.pose.position.x = max([min([0.1 * data.pose.position.x, position_limits[0][1]]), position_limits[0][0]])
    robot_pose.pose.position.y = max([min([0.1 * data.pose.position.y, position_limits[1][1]]), position_limits[1][0]])
    robot_pose.pose.position.z = max([min([0.1 * data.pose.position.z, position_limits[2][1]]), position_limits[2][0]])
    robot_pose.pose.orientation = data.pose.orientation

    rospy.loginfo("robot_pose: " + str(robot_pose.pose.position.x) + " " + str(robot_pose.pose.position.y) + " " + str(robot_pose.pose.position.z))

    object_info = data.header.frame_id
    rospy.loginfo("object info:" + object_info + ", 1: " + object_info[0:1])

    if object_info[0:1] == "s" and isDynamic_old:
        # set stiffness high
        client.update_configuration({'translational_stiffness': 1000.0, 'rotational_stiffness': 30.0})
        isDynamic_old = False

    elif object_info[0:1] == "d" and not isDynamic_old:
        # set stiffness low
        client.update_configuration({'translational_stiffness': 50.0, 'rotational_stiffness': 10.0})
        isDynamic_old = True

    roughness_level = object_info[1:2]

    new_data = True

    int_marker.pose = robot_pose.pose
    server.setPose("equilibrium_pose", robot_pose.pose)
    server.applyChanges()


def process_feedback(feedback):
    if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
        robot_pose.pose.position.x = max([min([feedback.pose.position.x, position_limits[0][1]]), position_limits[0][0]])
        robot_pose.pose.position.y = max([min([feedback.pose.position.y, position_limits[1][1]]), position_limits[1][0]])
        robot_pose.pose.position.z = max([min([feedback.pose.position.z, position_limits[2][1]]), position_limits[2][0]])
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

    rospy.loginfo("current position: " + str(robot_pose.pose.position.x) + ", " + str(robot_pose.pose.position.y) + ", " + str(robot_pose.pose.position.z) )
    rospy.loginfo("current orientation: " + str(robot_pose.pose.orientation.x) + ", " + str(robot_pose.pose.orientation.y) + ", " + str(robot_pose.pose.orientation.z)+  ", " + str(robot_pose.pose.orientation.w) )

def makeBox( msg ):
    marker = Marker()
    marker.type = Marker.CUBE
    marker.scale.x = msg.scale * 0.3
    marker.scale.y = msg.scale * 0.3
    marker.scale.z = msg.scale * 0.01
    marker.color.r = 0.5
    marker.color.g = 0.5
    marker.color.b = 0.5
    marker.color.a = 1.0
    return marker


if __name__ == "__main__":
    rospy.init_node("robot_manipulation")
    listener = tf.TransformListener()
    link_name = rospy.get_param("~link_name") #panda_link0

    wait_for_initial_pose()

    isContact = False
    global client
    global int_marker
    client = dynamic_reconfigure.client.Client(param_node, timeout=30, config_callback=param_callback)

    pose_pub = rospy.Publisher("equilibrium_pose", PoseStamped, queue_size=10)
    pose_sub = rospy.Subscriber("/cp_man", PoseStamped, robot_callback)
    force_sub = rospy.Subscriber("/franka_state_controller/F_ext", WrenchStamped, force_callback)

    server = InteractiveMarkerServer("equilibrium_pose_marker")

    int_marker = InteractiveMarker()
    int_marker.header.frame_id = link_name
    int_marker.scale = 0.5
    int_marker.name = "equilibrium_pose"
    int_marker.pose = robot_pose.pose
    # run pose publisher
    rospy.Timer(rospy.Duration(0.005), lambda msg: publisher_callback(msg, link_name))

    # attach box
    control = InteractiveMarkerControl()
    control.always_visible = True
    control.markers.append(makeBox(int_marker))
    control.interaction_mode = InteractiveMarkerControl.MOVE_ROTATE_3D
    int_marker.controls.append(control)

    server.insert(int_marker, process_feedback)
    server.applyChanges()
    rospy.spin()


