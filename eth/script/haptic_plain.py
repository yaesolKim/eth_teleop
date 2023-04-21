#!/usr/bin/env python
import rospy
import tf.transformations
import numpy as np
import dynamic_reconfigure.client

from interactive_markers.interactive_marker_server import InteractiveMarkerServer, InteractiveMarkerFeedback
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl
from geometry_msgs.msg import PoseStamped, WrenchStamped, Point
from std_msgs.msg import Int16
from franka_msgs.msg import FrankaState

from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *

from datetime import datetime

haptic_pose = PoseStamped()
pose_pub = None
ethContact = False
telemanContact = False
freeNow = False

K_pub = None
K_eth = 0

param_node = '/cartesian_impedance_example_controllerdynamic_reconfigure_compliance_param_node'

# [[min_x, max_x], [min_y, max_y], [min_z, max_z]] # front/back, left/right, up/down
position_limits = [[0.0, 0.8], [-0.6, 0.6], [0.4, 0.9]]

def param_callback(config):
    rospy.loginfo("Config set")

def process_feedback(feedback):
    True
    #do nothing

def makeHandle(msg):
    marker = Marker()
    marker.type = 10
    marker.mesh_resource = "package://eth/description/door_handle.stl"
    marker.mesh_use_embedded_materials = True

    marker.scale.x = msg.scale * 0.2
    marker.scale.y = msg.scale * 0.2
    marker.scale.z = msg.scale * 0.2

    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0

    marker.color.r = 0.5
    marker.color.g = 0.5
    marker.color.b = 0.5
    marker.color.a = 1.0
    return marker

# Set robot contact state based on the external force
def eth_force_callback(data):
    global ethContact
    global client

    ths_force = 3.0
    ths_no_force = 2.0

    # rospy.loginfo("force: " + str(data.wrench.force.z))
    if data.wrench.force.z > ths_force:
        if not ethContact:
            rospy.loginfo("eth contact time: " + datetime.utcnow().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3])
        ethContact = True

    elif data.wrench.force.z < ths_no_force:
        if ethContact:
            client.update_configuration({'translational_stiffness': 800.0, 'rotational_stiffness': 30.0})
        ethContact = False
def teleman_force_callback(data):
    global K_eth
    global client
    global telemanContact

    #rospy.loginfo(data.wrench.force.z)

    if data.wrench.force.z < -0.2:
        if not telemanContact:
            rospy.loginfo("teleman contact time: " + datetime.utcnow().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3])
        telemanContact = True

    elif data.wrench.force.z > 0:
        telemanContact = False

    # contact: adjust stiffness depending on the teleman external force
#if ethContact:
    if data.wrench.force.z > -4.0:
        if K_eth != 10:
            rospy.loginfo("1")
            client.update_configuration({'translational_stiffness': 10.0, 'rotational_stiffness': 10.0})
        K_eth = 10
    elif data.wrench.force.z > -8.0:
        if K_eth != 40:
            rospy.loginfo("2")
            client.update_configuration({'translational_stiffness': 40.0, 'rotational_stiffness': 10.0})
        K_eth = 40
    elif data.wrench.force.z > -12.0:
        if K_eth != 70:
            rospy.loginfo("3")
            client.update_configuration({'translational_stiffness': 70.0, 'rotational_stiffness': 10.0})
        K_eth = 70
    else:
        if K_eth != 100:
            rospy.loginfo("4")
            client.update_configuration({'translational_stiffness': 100.0, 'rotational_stiffness': 10.0})
        K_eth = 100

def pose_callback(data):
    # set haptic pose in the limit
    haptic_pose.pose.position.x = max([min([0.1 * data.pose.position.x, position_limits[0][1]]), position_limits[0][0]])
    haptic_pose.pose.position.y = max([min([0.1 * data.pose.position.y, position_limits[1][1]]), position_limits[1][0]])
    haptic_pose.pose.position.z = max([min([0.1 * data.pose.position.z, position_limits[2][1]]), position_limits[2][0]])

    # fix the pose as door handle
    haptic_pose.pose.orientation.x = 0.7071068
    haptic_pose.pose.orientation.y = 0.000
    haptic_pose.pose.orientation.z = 0.7071068
    haptic_pose.pose.orientation.w = 0.000

    server.setPose("equilibrium_pose", haptic_pose.pose)
    server.applyChanges()

    if not ethContact:
        rospy.loginfo("pose update")
        pose_pub.publish(haptic_pose)

    K_pub.publish(K_eth)

def wait_for_initial_pose():
    msg = rospy.wait_for_message("franka_state_controller/franka_states", FrankaState)  # type: FrankaState

    initial_quaternion = tf.transformations.quaternion_from_matrix(np.transpose(np.reshape(msg.O_T_EE, (4, 4))))
    initial_quaternion = initial_quaternion / \
        np.linalg.norm(initial_quaternion)

    haptic_pose.pose.orientation.x = initial_quaternion[0]
    haptic_pose.pose.orientation.y = initial_quaternion[1]
    haptic_pose.pose.orientation.z = initial_quaternion[2]
    haptic_pose.pose.orientation.w = initial_quaternion[3]
    haptic_pose.pose.position.x = msg.O_T_EE[12]
    haptic_pose.pose.position.y = msg.O_T_EE[13]
    haptic_pose.pose.position.z = msg.O_T_EE[14]

    rospy.loginfo("init pos: " + str(haptic_pose.pose.position) + ", init ori: " + str(haptic_pose.pose.orientation))

if __name__ == "__main__":
    rospy.init_node("robot_haptic")
    listener = tf.TransformListener()
    link_name = rospy.get_param("~link_name") #panda_link0
    wait_for_initial_pose()

    global client
    client = dynamic_reconfigure.client.Client(param_node, timeout=30, config_callback=param_callback)
    client.update_configuration({'translational_stiffness': 500.0, 'rotational_stiffness': 30.0})

    pose_pub = rospy.Publisher("equilibrium_pose", PoseStamped, queue_size=1)
    pose_sub = rospy.Subscriber("/cp_haptic", PoseStamped, pose_callback)

    eth_force_sub = rospy.Subscriber("/franka_state_controller/F_ext", WrenchStamped, eth_force_callback)
    teleman_force_sub = rospy.Subscriber("teleman/F_ext", WrenchStamped, teleman_force_callback)

    K_pub = rospy.Publisher("K_eth", Int16, queue_size=1)
    K_pub.publish(K_eth)

    server = InteractiveMarkerServer("equilibrium_pose_marker")

    int_marker = InteractiveMarker()
    int_marker.header.frame_id = link_name
    int_marker.scale = 0.5
    int_marker.name = "equilibrium_pose"
    int_marker.pose = haptic_pose.pose

    haptic_pose.header.frame_id = link_name

    control = InteractiveMarkerControl()
    control.always_visible = True
    control.markers.append(makeHandle(int_marker))
    control.interaction_mode = InteractiveMarkerControl.MOVE_ROTATE_3D
    int_marker.controls.append(control)

    server.insert(int_marker, process_feedback)
    server.applyChanges()

    rospy.spin()
