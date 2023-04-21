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

from std_msgs.msg import String
from datetime import datetime

haptic_pose = PoseStamped()
pose_pub = None
ethContact = False
teleopContact = False
freeNow = False
K_eth = 0
K_reconf = 1

param_node = '/cartesian_impedance_example_controllerdynamic_reconfigure_compliance_param_node'

# [[min_x, max_x], [min_y, max_y], [min_z, max_z]] # front/back, left/right, up/down
position_limits = [[0.0, 0.8], [0.3, 0.7], [0.4, 0.9]]

def param_callback(config):
    True
#    rospy.loginfo("Config set")

def process_feedback(feedback):
    True
    #do nothing

def next_trial_callback(data):
    rospy.loginfo("next trial callback")
    haptic_pose.pose.position.x = 0.4068714935406904
    haptic_pose.pose.position.y = 0.5845123228375686
    haptic_pose.pose.position.z = 0.5367608135925555
    pose_pub.publish(haptic_pose)
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
    global K_reconf

    ths_force = 3.0
    ths_no_force = 2.0

    if data.wrench.force.z > ths_force:
        if not ethContact:
            rospy.loginfo("eth make contact: " + datetime.utcnow().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3])
        ethContact = True

        if K_eth == 0:
            if K_reconf != 0:
                client.update_configuration({'translational_stiffness': 20.0, 'rotational_stiffness': 30.0})
            K_reconf = 0
        elif K_eth == 10:
            if K_reconf != 10:
                client.update_configuration({'translational_stiffness': 30.0, 'rotational_stiffness': 10.0})
            K_reconf = 10
        elif K_eth == 40:
            if K_reconf != 40:
                client.update_configuration({'translational_stiffness': 40.0, 'rotational_stiffness': 10.0})
            K_reconf = 40
        elif K_eth == 70:
            if K_reconf != 70:
                client.update_configuration({'translational_stiffness': 50.0, 'rotational_stiffness': 10.0})
            K_reconf = 70
        else:
            if K_reconf != 100:
                client.update_configuration({'translational_stiffness': 60.0, 'rotational_stiffness': 10.0})
            K_reconf = 100

    elif data.wrench.force.z < ths_no_force:
        if ethContact:
            rospy.loginfo("eth finish contact: " + datetime.utcnow().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3])
            client.update_configuration({'translational_stiffness': 800.0, 'rotational_stiffness': 20.0})
        K_reconf = 800
        ethContact = False

def teleman_force_callback(data):
    global teleopContact
    global K_eth

    #rospy.loginfo(data.wrench.force.z)
    F_ext_teleop = data.wrench.force.z

    if F_ext_teleop < -0.2:
        if not teleopContact:
            rospy.loginfo("teleop start contact: " + datetime.utcnow().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3])
        teleopContact = True
    elif F_ext_teleop > 0:
        if teleopContact:
            rospy.loginfo("teleop finish contact: " + datetime.utcnow().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3])
        teleopContact = False

    # contact: adjust stiffness depending on the teleman external force
    if F_ext_teleop > 0:
        K_eth = 0
    elif F_ext_teleop > -3.0:
        K_eth = 10
    elif F_ext_teleop > -5.0:
        K_eth = 40
    elif F_ext_teleop > -10.0:
        K_eth = 70
    else:
        K_eth = 100


def pose_callback(data):
    global ethContact

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
        pose_pub.publish(haptic_pose)

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
    client.update_configuration({'translational_stiffness': 800.0, 'rotational_stiffness': 30.0})

    pose_pub = rospy.Publisher("equilibrium_pose", PoseStamped, queue_size=1)
    pose_sub = rospy.Subscriber("/cp_haptic", PoseStamped, pose_callback)

    eth_force_sub = rospy.Subscriber("/franka_state_controller/F_ext", WrenchStamped, eth_force_callback)
    teleman_force_sub = rospy.Subscriber("teleman/F_ext", WrenchStamped, teleman_force_callback)

    next_trial_sub = rospy.Subscriber("/next_trial", String, next_trial_callback)

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
