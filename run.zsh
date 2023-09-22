#!/usr/bin/zsh

#echo "Start F/T sensor nodes!"
#/bin/zsh -ec 'cd /home/vicarios/PycharmProjects/eth_teleop && source devel/setup.zsh && python ./src/wittenstein_hex10/Resense.py/src/HEX10.py & '
#sleep 2

echo "Put the condition number (1-low K, 2-high K, 3-h_vic, 4-a_vic) :"

read varname
echo ETH and Teleoperation starts for $varname

/bin/zsh -ec 'cd /home/vicarios/PycharmProjects/eth_teleop && source devel/setup.zsh && roslaunch eth_teleman robot_eth.launch robot_ip:=10.240.14.24 control:=a_vic & '
sleep 5
/bin/zsh -ec 'cd /home/vicarios/PycharmProjects/eth_teleop && source devel/setup.zsh && roslaunch eth_teleman robot_rem.launch robot_ip:=10.240.14.25 control:=a_vic & '
sleep 5
/bin/zsh -ec 'cd /home/vicarios/PycharmProjects/eth_teleop && source devel/setup.zsh && rosrun eth_teleman data_recorder.py & '
sleep 2

# condition 1 : with low static K
if (($varname == 1 )); then
echo "Contion 1: ETH-teleoperation with low static stiffness!"
/bin/zsh -ec 'cd /home/vicarios/PycharmProjects/eth_teleop && source devel/setup.zsh && rosrun rosbag record -O low_static_K /data_for_recording & '
sleep 2
fi

# condition 2 : with high static K
if (($varname == 2 )); then
echo "Contion 2: ETH-teleoperation with with high static stiffness!"
/bin/zsh -ec 'cd /home/vicarios/PycharmProjects/eth_teleop && source devel/setup.zsh && rosrun rosbag record -O high_static_K /data_for_recording & '
sleep 2
fi


# condition 3 : with heuristic VIC, h_vic
if (($varname == 3 )); then
echo "Contion 3: ETH-teleoperation with heuristic variable stiffness control!"
/bin/zsh -ec 'cd /home/vicarios/PycharmProjects/eth_teleop && source devel/setup.zsh && rosrun rosbag record -O heuristic_K /data_for_recording & '
sleep 2
#sleep 2
fi

# condition 4 : with algorithmic VIC, a_vic
if (($varname == 4 )); then
echo "Contion 4: ETH-teleoperation with algorithmic variable stiffness control!"
/bin/zsh -ec 'cd /home/vicarios/PycharmProjects/eth_teleop && source devel/setup.zsh && rosrun rosbag record -O algorithmic_K /data_for_recording & '
sleep 2
fi


# For all conditions
echo "Automatic error recovering starts"
/bin/zsh -ec 'cd /home/vicarios/PycharmProjects/eth_teleop && source devel/setup.zsh && rosrun eth_teleman eth_error_recover & '
/bin/zsh -ec 'cd /home/vicarios/PycharmProjects/eth_teleop && source devel/setup.zsh && rosrun eth_teleman rem_error_recover & '
sleep 2

echo "If you want to exit, put 9"
read varname
if (($varname == 9 )); then
exit 1