#!/usr/bin/zsh
echo "Start F/T sensor nodes!"
/bin/zsh -ec 'cd /home/vicarios/PycharmProjects/teleop_ws && source devel/setup.zsh && python ./src/wittenstein_hex10/Resense.py/src/HEX10.py & '
sleep 5

echo "Put the condition number (0, 1, 2, 3) :"

read varname
echo ETH and Teleoperation starts for $varname


# training
if (($varname == 0 )); then
echo "training: ETH only, no teleoperation"
/bin/zsh -ec 'cd /home/vicarios/PycharmProjects/eth_ws && source devel/setup.zsh && roslaunch eth eth_robot.launch robot:=real control:=topic2 & '
sleep 5

echo "Automatic error recovering starts"
/bin/zsh -ec 'cd /home/vicarios/PycharmProjects/eth_ws && source devel/setup.zsh && rosrun eth error_recovery_node & '
sleep 5

fi

# condition 1 : no eth
if (($varname == 1 )); then
echo "Contion 1: No ETH, teleoperation only"
/bin/zsh -ec 'cd /home/vicarios/PycharmProjects/teleop_ws && source devel/setup.zsh && roslaunch teleman teleman_robot.launch control:=topic1 & '
sleep 5

echo "Automatic error recovering starts"
/bin/zsh -ec 'cd /home/vicarios/PycharmProjects/eth_ws && source devel/setup.zsh && rosrun eth error_recovery_node & '
sleep 5

echo "Record eth and teleop topics"
/bin/zsh -ec 'cd /home/vicarios/PycharmProjects/eth_ws && source devel/setup.zsh && rosrun rosbag record -O 1eth /cartesian_impedance_example_controller/equilibrium_pose /franka_state_controller/F_ext /cp_haptic /cartesian_impedance_example_controllerdynamic_reconfigure_compliance_param_node/parameter_updates & '
/bin/zsh -ec 'cd /home/vicarios/PycharmProjects/eth_ws && source devel/setup.zsh && rosrun rosbag record -O 1teleop /teleman/cartesian_impedance_example_controller/equilibrium_pose /teleman/franka_state_controller/F_ext teleman/F_ext /teleop_franka /teleman/cartesian_impedance_example_controllerdynamic_reconfigure_compliance_param_node/parameter_updates & '
/bin/zsh -ec 'cd /home/vicarios/PycharmProjects/eth_ws && source devel/setup.zsh && rosrun rosbag record -O 1eth_states /franka_state_controller/franka_states & '
/bin/zsh -ec 'cd /home/vicarios/PycharmProjects/eth_ws && source devel/setup.zsh && rosrun rosbag record -O 1teleop_states /teleman/franka_state_controller/franka_states & '

fi


# condition 2 : eth with Ks
if (($varname == 2 )); then
echo "Contion 2: ETH teleoperation with static stiffness!"
/bin/zsh -ec 'cd /home/vicarios/PycharmProjects/eth_ws && source devel/setup.zsh && roslaunch eth eth_robot.launch robot:=real control:=topic2 & '
sleep 5
/bin/zsh -ec 'cd /home/vicarios/PycharmProjects/teleop_ws && source devel/setup.zsh && roslaunch teleman teleman_robot.launch control:=topic1 & '
sleep 5

echo "Automatic error recovering starts"
/bin/zsh -ec 'cd /home/vicarios/PycharmProjects/eth_ws && source devel/setup.zsh && rosrun eth error_recovery_node & '
sleep 5

echo "Record eth and teleop topics"
/bin/zsh -ec 'cd /home/vicarios/PycharmProjects/eth_ws && source devel/setup.zsh && rosrun rosbag record -O 2eth /cartesian_impedance_example_controller/equilibrium_pose /franka_state_controller/F_ext /cp_haptic /cartesian_impedance_example_controllerdynamic_reconfigure_compliance_param_node/parameter_updates & '
/bin/zsh -ec 'cd /home/vicarios/PycharmProjects/eth_ws && source devel/setup.zsh && rosrun rosbag record -O 2teleop /teleman/cartesian_impedance_example_controller/equilibrium_pose /teleman/franka_state_controller/F_ext teleman/F_ext /teleop_franka /teleman/cartesian_impedance_example_controllerdynamic_reconfigure_compliance_param_node/parameter_updates & '
/bin/zsh -ec 'cd /home/vicarios/PycharmProjects/eth_ws && source devel/setup.zsh && rosrun rosbag record -O 2eth_states /franka_state_controller/franka_states & '
/bin/zsh -ec 'cd /home/vicarios/PycharmProjects/eth_ws && source devel/setup.zsh && rosrun rosbag record -O 2teleop_states /teleman/franka_state_controller/franka_states & '

fi


# condition 3 : eth with Kd
if (($varname == 3 )); then
echo "Contion 3: ETH teleoperation with variable stiffness!"
/bin/zsh -ec 'cd /home/vicarios/PycharmProjects/eth_ws && source devel/setup.zsh && roslaunch eth eth_robot.launch robot:=real control:=topic3 & '
sleep 5
/bin/zsh -ec 'cd /home/vicarios/PycharmProjects/teleop_ws && source devel/setup.zsh && roslaunch teleman teleman_robot.launch control:=topic3 & '
sleep 5


echo "Automatic error recovering starts"
/bin/zsh -ec 'cd /home/vicarios/PycharmProjects/eth_ws && source devel/setup.zsh && rosrun eth error_recovery_node & '
sleep 5

echo "Record eth and teleop topics"
/bin/zsh -ec 'cd /home/vicarios/PycharmProjects/eth_ws && source devel/setup.zsh && rosrun rosbag record -O 3eth /cartesian_impedance_example_controller/equilibrium_pose /franka_state_controller/F_ext /cp_haptic /cartesian_impedance_example_controllerdynamic_reconfigure_compliance_param_node/parameter_updates & '
/bin/zsh -ec 'cd /home/vicarios/PycharmProjects/eth_ws && source devel/setup.zsh && rosrun rosbag record -O 3teleop /teleman/cartesian_impedance_example_controller/equilibrium_pose /teleman/franka_state_controller/F_ext teleman/F_ext /teleop_franka /teleman/cartesian_impedance_example_controllerdynamic_reconfigure_compliance_param_node/parameter_updates & '
/bin/zsh -ec 'cd /home/vicarios/PycharmProjects/eth_ws && source devel/setup.zsh && rosrun rosbag record -O 3eth_states /franka_state_controller/franka_states & '
/bin/zsh -ec 'cd /home/vicarios/PycharmProjects/eth_ws && source devel/setup.zsh && rosrun rosbag record -O 3teleop_states /teleman/franka_state_controller/franka_states & '

fi


echo "If you want to exit, put 9"
read varname
if (($varname == 9 )); then
exit 1