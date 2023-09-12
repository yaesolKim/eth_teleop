## Change logs
### 18/11/22   
* Changed robot position limit and change the goal to avoid the joint limit errors below
  * libfranka: Move command aborted: motion aborted by reflex! ["joint_position_limits_violation"]
  * libfranka: Move command aborted: motion aborted by reflex! ["self_collision_avoidance_violation"]
  
### 07/11/22   
* Changed eth robot to new one (third Franka)
  * System version: 4.2.2
  * IP address: 192.168.88.11   
* Changed franka end-effector setting in Franka Desk    

### ??/??/22
* Changed franka end-effector setting in Franka Desk    
  ![desk](/uploads/2f3696875811bd939fbb00d4dc76b421/desk.png)   
  * transformation: 0.043m in z direction
  * weight: 0.1kg   
    
### ??/??/22
* Change franka_example_controllers/cfg/compliance_param.cfg
  * Tried setParam but not works. Instead, dynamic reconfiguration package is used:   
  `client = dynamic_reconfigure.client.Client(param_node, timeout=30, config_callback=param_callback)`