#!/usr/bin/zsh

echo "Start F/T sensor nodes-------------"
#/bin/zsh -ec 'cd /home/vicarios/PycharmProjects/eth_teleop && source devel/setup.zsh && roslaunch netft_rdt_driver ft_sensor.launch & '
#sleep 2
#sudo chmod +777 /dev/ttyACM*
/bin/zsh -ec 'cd /home/vicarios/PycharmProjects/eth_teleop && source devel/setup.zsh && python ./src/wittenstein_hex10/Resense.py/src/HEX10.py & '
sleep 2

echo "eth nodes--------------------------"
/bin/zsh -ec 'cd /home/vicarios/PycharmProjects/eth_teleop && source devel/setup.zsh && roslaunch eth eth_robot.launch robot:=real control:=ke & '
sleep 5
/bin/zsh -ec 'cd /home/vicarios/PycharmProjects/eth_teleop && source devel/setup.zsh && rosrun eth eth_error_recovery_node & '
sleep 2

echo "teleman nodes---------------------"
/bin/zsh -ec 'cd /home/vicarios/PycharmProjects/eth_teleop && source devel/setup.zsh && roslaunch teleman teleman_robot.launch robot:=real control:=ke & '
sleep 5
/bin/zsh -ec 'cd /home/vicarios/PycharmProjects/eth_teleop && source devel/setup.zsh && rosrun teleman teleman_error_recovery_node & '
sleep 2

echo "Record topics----------------------"
/bin/zsh -ec 'cd /home/vicarios/PycharmProjects/eth_teleop && source devel/setup.zsh && rosrun teleman record_forces.py & '
sleep 2
/bin/zsh -ec 'cd /home/vicarios/PycharmProjects/eth_teleop && source devel/setup.zsh && rosrun rosbag record -O vic /data_P & '
#/teleman/franka_state_controller/franka_states /franka_state_controller/franka_states
sleep 2

#echo "Record topics"
#/bin/zsh -ec 'cd /home/vicarios/PycharmProjects/eth_teleop && source devel/setup.zsh && rosrun rosbag record -O ethh2 /K_e /CIMP_eth_controller/equilibrium_pose /cp_haptic /franka_state_controller/F_ext /franka_state_controller/franka_states  & '
#sleep 2
#/bin/zsh -ec 'cd /home/vicarios/PycharmProjects/eth_teleop && source devel/setup.zsh && rosrun rosbag record -O remoteh2 /teleman/CIMP_teleman_controller/K_e /teleman/CIMP_teleman_controller/equilibrium_pose /teleman/F_ext /teleman/franka_state_controller/franka_states  & '
#sleep 2
#/bin/zsh -ec 'cd /home/vicarios/PycharmProjects/eth_teleop && source devel/setup.zsh && rosrun rosbag record -O cic_60 /teleman/F_ext /franka_state_controller/F_ext /teleman/CIMP_teleman_controller/K_e & '

echo "If you want to exit, put 9"
read varname
if (($varname == 9 )); then
exit 1