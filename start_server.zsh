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

echo "If you want to exit, put 9"
read varname
if (($varname == 9 )); then
exit 1