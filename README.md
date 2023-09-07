# Bilateral ETH-Teleoperation   
Scripts to run the Encountered-type Haptic Teleoperation system.   

## How to Run   
1. Launch ros server: `./start_server.zsh`   
2. To use HEX10 FT sensor: `chmod *777 /dev/ttyACM*`   
3. Run script for launch everything: `./h_vic.zsh`    

---------------------

## About each scripts
1. define_Ks.zsh   
I used to collect the data for defining the static K value (for SMC23).   

2. h_vic.zsh    
Core script to run the system.   

3. start_server.zsh   
Script that needs to be executed first.   

4. user_study.zsh   
Script that I used for main user study (for SMC23).   


## Private Repositories   
- [eth_teleman_franka](https://github.com/yaesolKim/eth_teleman_franka)   
- [franka_ros](https://github.com/yaesolKim/franka_ros)    


## Public Repositories   
- [dynamic_reconfigure](https://github.com/ros/dynamic_reconfigure)   
