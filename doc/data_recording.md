## Record the DATA!
### 12/SEP/23   
* record data in one bag file to analyze it in same timeline!   



### Terminal 1
1. Launch ros server: `./start_server.zsh`   

### Terminal 2
1. To use HEX10 FT sensor: `chmod *777 /dev/ttyACM*`   
2. Run script for launch everything: `./run.zsh`    
3. Select the control method: (1) for h_vic, (2) for a_vic.     

### Terminal 3   
1. run test condition script: `rosrun eth_teleman test_condition.py`   
2. Select the control method: (0) for move to initial pose, (1) for using static low K, (2) for using static high K, (3) for a_vic.


1. record low statik K for 10 times  
T2 -> (2)  
T3 -> (0) -> (1) 10 times  
 
2. record high statik K for 10 times  
T2 -> (2)  
T3 -> (0) -> (2) 10 times  

3. record heuristic K for 10 times  
T2 -> (1)  
T3 -> (0) -> (0) 10 times  

4. record algorithmic K for 10 times  
T2 -> (2)  
T3 -> (0) -> (3) 10 times  