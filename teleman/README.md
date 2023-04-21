# ETH Tele-manipulation: Franka Emika and Hannes
Project for encountered-type haptic tele-manipulation.

* Subscribe cartesian pose from Unreal using ros_bridge
* Motion planning using MoveIt!
* Publish planned path to Franka robot   
* Reconfigure the stiffness in runtime    

## How to Run  
Launch the following command to start ros bridge server and launch other commands each in a new terminal

`roslaunch rosbridge_server rosbridge_websocket.launch websocket_external_port:=80`     


### ETH Tele-manipulation with real robot

1. Run tele-manipulation controller     
`roslaunch teleman teleman_control.launch mode:=[topic|marker]`
2. `rosrun hannes_hand tele_grasp`
<!--3. or `rosrun hannes_hand teleman_grasp_client` -->
 
- If there exist error from (1), about /ttyUSB0, try the command below  
`sudo chown vicarios /dev/ttyUSB0`  


<!--

### Tele-manipulation simulation with gazebo      

1. `roslaunch teleman panda_gazebo.launch controller:=cartesian_impedance_example_controller`   
2. `roslaunch teleman teleman_robot.launch mode:=[man|marker]`   
3. `rosrun teleman test_publisher.py`



### Tele-manipulation with moveit!

1. Run tele-manipulation controller     
`roslaunch teleman teleman_control.launch robot_ip:=192.168.1.9` 
#mode:=[vr|marker]` arm_id:=teleman

2. `roslaunch panda_simulation simulation.launch`   
3. `rosrun hannes_hand tele_grasp`  
4. `rosrun teleman telemanipulation`    

### Tele-manipulation simulation with gazebo using moveit!   

1. `roslaunch teleman panda_gazebo.launch`   
2. `roslaunch panda_simulation simulation.launch`   
3. `rosrun teleman telemanipulation`

?? roslaunch panda_moveit_config demo_gazebo.launch   
-->

---------------------

## Requirements  
- Ubuntu 20.04
- Ros Noetic
- Franka system : version 4.2.2
- [libfranka](https://github.com/frankaemika/libfranka) : version 0.9.0    
- [franka_ros](https://github.com/frankaemika/franka_ros) : version 0.9.0
- [controller](https://gitlab.iit.it/vicarios/robots/franka/controllers) : from VICARIOS Lab
- [dynamic_reconfigure](https://github.com/ros/dynamic_reconfigure)
- [hannes_hand]()


<!--
- [URComm: ROS bridge](https://gitlab.iit.it/vicarios/unreal/plugins/URComm)
-->