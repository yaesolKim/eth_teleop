#!/usr/bin/zsh

pid=$(sudo lsof -n -i :9090 | grep LISTEN)
pid=${pid:7:7}
echo Process PID $pid
echo Closing 9090 port with $pid
kill -9 $pid

pid=$(pgrep roslaunch)
echo $pid
echo Closing roslaunch
kill -9 $pid

echo "Run ROSBridge server!"
/bin/zsh -ec 'roslaunch rosbridge_server rosbridge_websocket.launch websocket_external_port:=80 & '
sleep 5

echo "Put the condition number (0 for Ks_eth, 1 for Ks_teleop) :"

read varname
echo ETH and Teleoperation starts for $varname


# find Ks_eth
if (($varname == 1 )); then
echo "haptic3 teleop1"
/bin/zsh -ec 'cd /home/vicarios/PycharmProjects/eth_ws && source devel/setup.zsh && roslaunch eth eth_robot.launch robot:=real control:=topic3 & '
sleep 5
echo "Start Teleoperation nodes!"
/bin/zsh -ec 'cd /home/vicarios/PycharmProjects/teleop_ws && source devel/setup.zsh && roslaunch teleman teleman_robot.launch control:=topic1 & '
sleep 5
echo "Start F/T sensor nodes!"
/bin/zsh -ec 'cd /home/vicarios/PycharmProjects/teleop_ws && source devel/setup.zsh && python ./src/wittenstein_hex10/Resense.py/src/HEX10.py & '
sleep 5
echo "start recording rosbag!"
/bin/zsh -ec 'cd /home/vicarios/PycharmProjects/eth_ws && source devel/setup.zsh && cd src/rosbag_to_csv/bags && rosrun rosbag record -O F4 teleman/F_ext & '
sleep 5
fi

# find Ks_teleop
if (($varname == 2 )); then
echo "training: ETH nodes only, no teleoperation nodes"
/bin/zsh -ec 'cd /home/vicarios/PycharmProjects/eth_ws && source devel/setup.zsh && roslaunch eth eth_robot.launch robot:=real control:=topic2 & '
sleep 5
echo "start recording rosbag!"
/bin/zsh -ec 'cd /home/vicarios/PycharmProjects/eth_ws && source devel/setup.zsh && cd src/rosbag_to_csv/bags && rosrun rosbag record -O F5 /franka_state_controller/F_ext & '
sleep 5
fi



echo "automatic error recovering starts"
/bin/zsh -ec 'cd /home/vicarios/PycharmProjects/eth_ws && source devel/setup.zsh && rosrun eth error_recovery_node & '
sleep 5



echo "If you want to exit, put 9"
read varname
if (($varname == 9 )); then
exit 1