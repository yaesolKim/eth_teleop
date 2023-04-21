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

import math
import datetime

haptic_pose = PoseStamped()
pose_pub = None
new_data = False

isContact = False
wasContact = False

param_node = '/cartesian_impedance_example_controllerdynamic_reconfigure_compliance_param_node'
#param_node = '/cartesian_admittance_controllerdynamic_reconfigure_compliance_param_node'

# [[min_x, max_x], [min_y, max_y], [min_z, max_z]] # front/back, left/right, up/down
position_limits = [[0.2, 0.8], [-0.6, 0.6], [0.05, 0.9]]

def param_callback(config):
    rospy.loginfo("Config set")

# compare to the current pose, set the goal position again
def setShortGoalPosition(position):
    msg = rospy.wait_for_message("franka_state_controller/franka_states", FrankaState)  # type: FrankaState
    current_x = msg.O_T_EE[12]
    current_y = msg.O_T_EE[13]
    current_z = msg.O_T_EE[14]

    goalPosition = position

    dist = math.sqrt((current_x - position.x)**2 + (current_y - position.y)**2 + (current_z - position.z)**2)
    if dist > 0.3:
        goalPosition.x = current_x + (position.x - current_x) / dist * 0.3
        goalPosition.y = current_y + (position.y - current_y) / dist * 0.3
        goalPosition.z = current_z + (position.z - current_z) / dist * 0.3

        rospy.loginfo("far!")
        rospy.loginfo("haptic_pose current: " + str(current_x) + " " + str(current_y) + " " + str(current_z))
        rospy.loginfo("haptic_pose after: " + str(goalPosition))

    return goalPosition

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

def makeBox(msg):
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


# Set robot contact state based on the external force
def force_callback(data):
    global isContact
    ths_force = 3.0
    ths_no_force = 2.0

    # rospy.loginfo("force: " + str(data.wrench.force.z))

    if data.wrench.force.z > ths_force:
        isContact = True
        ## print("eth contact!" + datetime.datetime.now())
        # rospy.loginfo("contact")

    elif data.wrench.force.z < ths_no_force:
        isContact = False
        #rospy.loginfo("no contact")


#set contact configuration: contact surface pose(position and orientation), object property(static/dynamic), surface property(level of textures)
def haptic_callback(data):
    global isContact
    global wasContact
    global client

    # set haptic pose in the limit
    haptic_pose.pose.position.x = max([min([0.1 * data.pose.position.x, position_limits[0][1]]), position_limits[0][0]])
    haptic_pose.pose.position.y = max([min([0.1 * data.pose.position.y, position_limits[1][1]]), position_limits[1][0]])
    haptic_pose.pose.position.z = max([min([0.1 * data.pose.position.z, position_limits[2][1]]), position_limits[2][0]])

    # haptic_pose.pose.position = setShortGoalPosition(haptic_pose.pose.position)

    # fix the pose as door handle
    haptic_pose.pose.orientation.x = 0.7071068
    haptic_pose.pose.orientation.y = 0.000
    haptic_pose.pose.orientation.z = 0.7071068
    haptic_pose.pose.orientation.w = 0.000

    #haptic_pose.pose.orientation = data.pose.orientation

    # static/dynamic, roughness level
    object_info = data.header.frame_id
    rospy.loginfo("object info:" + object_info + ", was/is Contact:" + str(wasContact) + "/" + str(isContact))

    # if object_info[0:1] == "s": # do nothing.

    if object_info[0:1] == "d":
        if not wasContact and isContact:
            client.update_configuration({'translational_stiffness': 50.0, 'rotational_stiffness': 10.0})
            rospy.loginfo("change stiff low")
            wasContact = True

        elif wasContact and not isContact:
            client.update_configuration({'translational_stiffness': 1000.0, 'rotational_stiffness': 30.0})
            rospy.loginfo("change stiff high")
            wasContact = False

    if not wasContact and isContact:
        wasContact = True

    new_data = True

    server.setPose("equilibrium_pose", haptic_pose.pose)
    server.applyChanges()

    # from publisher callback function
    haptic_pose.header.stamp = rospy.Time(0)
    # if there is no contact, follow contact predicted pose
    if not isContact:
        pose_pub.publish(haptic_pose)
    # hold the configuration.
    # else:
    #    rospy.loginfo("contact")



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
    client.update_configuration({'translational_stiffness': 1000.0, 'rotational_stiffness': 30.0})

    pose_pub = rospy.Publisher("equilibrium_pose", PoseStamped, queue_size=1)
    pose_sub = rospy.Subscriber("/cp_haptic", PoseStamped, haptic_callback)
    force_sub = rospy.Subscriber("/franka_state_controller/F_ext", WrenchStamped, force_callback)

    server = InteractiveMarkerServer("equilibrium_pose_marker")

    int_marker = InteractiveMarker()
    int_marker.header.frame_id = link_name
    int_marker.scale = 0.5
    int_marker.name = "equilibrium_pose"
    int_marker.pose = haptic_pose.pose

    haptic_pose.header.frame_id = link_name

    control = InteractiveMarkerControl()
    control.always_visible = True
    #control.markers.append(makeBox(int_marker))
    control.markers.append(makeHandle(int_marker))
    control.interaction_mode = InteractiveMarkerControl.MOVE_ROTATE_3D
    int_marker.controls.append(control)

    server.insert(int_marker, process_feedback)
    server.applyChanges()

    rospy.spin()
