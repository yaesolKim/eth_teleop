1.

Exception in thread /franka_state_controller/franka_states:
Traceback (most recent call last):
  File "/usr/lib/python3.8/threading.py", line 932, in _bootstrap_inner
    self.run()
  File "/usr/lib/python3.8/threading.py", line 870, in run
    self._target(*self._args, **self._kwargs)
  File "/opt/ros/noetic/lib/python3/dist-packages/rospy/impl/tcpros_pubsub.py", line 185, in robust_connect_subscriber
    conn.receive_loop(receive_cb)           
  File "/opt/ros/noetic/lib/python3/dist-packages/rospy/impl/tcpros_base.py", line 846, in receive_loop
[INFO] [1668779145.888539]: object info:d1, 1: d
    self.close()
  File "/opt/ros/noetic/lib/python3/dist-packages/rospy/impl/tcpros_base.py", line 858, in close
    self.socket.close()
AttributeError: 'NoneType' object has no attribute 'close'



2.


use moveit to plan the path again when one of the joint configuration is close to the limit

Now the new problem is... EE pose is not matched to the virtual surface...              
We should let the user know if the object is touchable or not.          
Using distance between user might be enough.

3. when stiffness is low -> Robot moves too slowly