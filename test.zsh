#!/usr/bin/zsh

echo "Start F/T sensor nodes-------------"
#/bin/zsh -ec 'cd /home/vicarios/PycharmProjects/eth_teleop && source devel/setup.zsh && roslaunch netft_rdt_driver ft_sensor.launch & '
#sleep 2
#sudo chmod +777 /dev/ttyACM*

/bin/zsh -ec 'cd /home/vicarios/PycharmProjects/eth_teleop && source devel/setup.zsh && python ./src/wittenstein_hex10/Resense.py/src/HEX10.py & '
sleep 2

echo "eth nodes--------------------------"
/bin/zsh -ec 'cd /home/vicarios/PycharmProjects/eth_teleop && source devel/setup.zsh && roslaunch eth_teleman eth_robot.launch robot:=real control:=ke & '
sleep 5
/bin/zsh -ec 'cd /home/vicarios/PycharmProjects/eth_teleop && source devel/setup.zsh && rosrun eth_teleman eth_error_recover & '
sleep 2

echo "teleman nodes---------------------"
/bin/zsh -ec 'cd /home/vicarios/PycharmProjects/eth_teleop && source devel/setup.zsh && roslaunch eth_teleman teleman_robot.launch robot:=real control:=ke & '
sleep 5
/bin/zsh -ec 'cd /home/vicarios/PycharmProjects/eth_teleop && source devel/setup.zsh && rosrun eth_teleman rem_error_recover & '
sleep 2

echo "Record topics----------------------"
/bin/zsh -ec 'cd /home/vicarios/PycharmProjects/eth_teleop && source devel/setup.zsh && rosrun eth_teleman record_data.py & '
sleep 2
/bin/zsh -ec 'cd /home/vicarios/PycharmProjects/eth_teleop && source devel/setup.zsh && rosrun rosbag record -O vic /data_P & '
sleep 2

echo "If you want to exit, put 9"
read varname
if (($varname == 9 )); then
exit 1