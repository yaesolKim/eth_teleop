#!/usr/bin/zsh

echo "Start F/T sensor nodes!"
/bin/zsh -ec 'cd /home/vicarios/PycharmProjects/eth_teleop && source devel/setup.zsh && python ./src/wittenstein_hex10/Resense.py/src/HEX10.py & '
sleep 2

echo "Put the condition number (1, 2) :"

read varname
echo ETH and Teleoperation starts for $varname

# condition 1 : with heuristic VIC, h_vic
if (($varname == 1 )); then
echo "Contion 2: ETH-teleoperation with heuristic variable stiffness control!"
/bin/zsh -ec 'cd /home/vicarios/PycharmProjects/eth_teleop && source devel/setup.zsh && roslaunch eth_teleman robot_eth.launch robot_ip:=10.240.14.24 control:=h_vic & '
sleep 5
/bin/zsh -ec 'cd /home/vicarios/PycharmProjects/eth_teleop && source devel/setup.zsh && roslaunch eth_teleman robot_rem.launch robot_ip:=10.240.14.25 control:=h_vic & '
sleep 5

echo "Record topics----------------------"
/bin/zsh -ec 'cd /home/vicarios/PycharmProjects/eth_teleop && source devel/setup.zsh && rosrun eth_teleman record_h_vic.py & '
sleep 2
/bin/zsh -ec 'cd /home/vicarios/PycharmProjects/eth_teleop && source devel/setup.zsh && rosrun rosbag record -O h_vic /data_P & '
sleep 2
#/bin/zsh -ec 'cd /home/vicarios/PycharmProjects/eth_ws && source devel/setup.zsh && rosrun rosbag record -O 3eth    /cp_haptic             /h_vic_eth_controller/equilibrium_pose /franka_state_controller/F_ext/wrench/force/z         /h_vic_eth_controller/EK_eth/data & '
#/bin/zsh -ec 'cd /home/vicarios/PycharmProjects/eth_ws && source devel/setup.zsh && rosrun rosbag record -O 3teleop /teleop_franka /teleman/h_vic_rem_controller/equilibrium_pose                 /teleman/F_ext/wrench/force/z /teleman/h_vic_rem_controller/EK_rem/data & '
#/bin/zsh -ec 'cd /home/vicarios/PycharmProjects/eth_ws && source devel/setup.zsh && rosrun rosbag record -O 3eth_states    /franka_state_controller/franka_states & '
#/bin/zsh -ec 'cd /home/vicarios/PycharmProjects/eth_ws && source devel/setup.zsh && rosrun rosbag record -O 3teleop_states /teleman/franka_state_controller/franka_states & '
fi

# condition 2 : with algorithmic VIC, a_vic
if (($varname == 2 )); then
echo "Contion 3: ETH-teleoperation with algorithamic variable stiffness control!"
/bin/zsh -ec 'cd /home/vicarios/PycharmProjects/eth_teleop && source devel/setup.zsh && roslaunch eth_teleman robot_eth.launch robot_ip:=10.240.14.24 control:=a_vic & '
sleep 5
/bin/zsh -ec 'cd /home/vicarios/PycharmProjects/eth_teleop && source devel/setup.zsh && roslaunch eth_teleman robot_rem.launch robot_ip:=10.240.14.25 control:=a_vic & '
sleep 5

echo "Record topics----------------------"
/bin/zsh -ec 'cd /home/vicarios/PycharmProjects/eth_teleop && source devel/setup.zsh && rosrun eth_teleman record_a_vic.py & '
sleep 2
/bin/zsh -ec 'cd /home/vicarios/PycharmProjects/eth_teleop && source devel/setup.zsh && rosrun rosbag record -O a_vic /data_P & '
sleep 2
fi


# For all conditions
echo "Automatic error recovering starts"
/bin/zsh -ec 'cd /home/vicarios/PycharmProjects/eth_teleop && source devel/setup.zsh && rosrun eth_teleman eth_error_recover & '
/bin/zsh -ec 'cd /home/vicarios/PycharmProjects/eth_teleop && source devel/setup.zsh && rosrun eth_teleman rem_error_recover & '
sleep 5




echo "If you want to exit, put 9"
read varname
if (($varname == 9 )); then
exit 1