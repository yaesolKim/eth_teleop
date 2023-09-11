# Bilateral ETH-Teleoperation   
Scripts to run the Encountered-type Haptic Teleoperation system.   

## How to Run   

### Terminal 1
1. Launch ros server: `./start_server.zsh`   

### Terminal 2
1. To use HEX10 FT sensor: `chmod *777 /dev/ttyACM*`   
2. Run script for launch everything: `./run.zsh`    
3. Select the control method: (1) for h_vic, (2) for a_vic.     

### Terminal 3   
1. run test condition script: `rosrun eth_teleman test_condition.py`   
2. Select the control method: (0) for move to initial pose, (1) for using static low K, (2) for using static high K, (3) for a_vic.    
---------------------

## About each scripts
1. define_Ks.zsh   
I used to collect the data for defining the static K value (for SMC23).   

2. run.zsh    
Core script to run the system.   

3. start_server.zsh   
Script that needs to be executed first.   

4. user_study.zsh   
Script that I used for main user study (for SMC23).   


---------------------

## Required sources

### Private Repositories   
- [eth_teleman_franka](https://github.com/yaesolKim/eth_teleman_franka)   
- [franka_ros](https://github.com/yaesolKim/franka_ros)    

### Public Repositories   
- [dynamic_reconfigure](https://github.com/ros/dynamic_reconfigure)   
