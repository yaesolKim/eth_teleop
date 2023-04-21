# ETH using Franka Emika Panda
ROS package for encountered-type haptic interface!   
* Subscribe contact predicted point from Unreal using ros_bridge
* Publish robot cartesian pose to Franka robot   
* Reconfigure the stiffness in runtime


## How to Run  
Launch the following commands to start ROSBrige server and ETH nodes. The default robot IP is set to 192.168.1.11.

1. `roslaunch rosbridge_server rosbridge_websocket.launch websocket_external_port:=80`     
2. `roslaunch eth eth_robot.launch robot:=[real|sim] control:=[topic|marker]`

**Arguments for ETH Ros nodes: _robot_ and _control_**  


| Argument for _robot_ | Which robot do you want to use? |
|----------------------|---------------------------------|
| real                 | Real Franka Emika robot         |
| sim                  | Simulated robot in gazebo       |

| Argument for _control_  | How to control the robot?     |
|-------------------------|-------------------------------|
| topic                   | By topic from [ETH VR system](https://gitlab.iit.it/vicarios/haptics/eth_unreal) or [test_publisher](https://gitlab.iit.it/vicarios/haptics/eth_ros/-/blob/master/script/test_publisher.py)|
| marker                  | By interactive marker in Rviz |

---------------------       

### ETH with real robot using moveit!

1. `roslaunch eth panda_control.launch robot_ip:=192.168.88.11`
2. `roslaunch panda_simulation simulation.launch`   
3. `rosrun eth haptic`   

### ETH simulation with gazebo using moveit!   

1. `roslaunch eth panda_gazebo.launch`   
2. `roslaunch panda_simulation simulation.launch`   
3. `rosrun eth haptic`

---------------------
  

https://github.com/AtsushiSakai/rosbag_to_csv

## Requirements  
- Ubuntu 20.04
- Ros Noetic
- Franka system : version 4.2.2
- [libfranka](https://github.com/frankaemika/libfranka) : version 0.9.0    
- [franka_ros](https://github.com/frankaemika/franka_ros) : version 0.9.0
- [controller](https://gitlab.iit.it/vicarios/robots/franka/controllers) : from VICARIOS Lab
- [dynamic_reconfigure](https://github.com/ros/dynamic_reconfigure)


<!--
- [URComm: ROS bridge](https://gitlab.iit.it/vicarios/unreal/plugins/URComm)
-->