### Multi Robot MPC  
Prerequisite Package: [Volta](https://github.com/botsync/volta)

#### Building the code    
Change gazebo model path by adding this line in ~/.bashrc  
* run ```sudo gedit ~/.bashrc```  
* add ```export GAZEBO_MODEL_PATH=/home/user_name/workspace_name/src/multi_robot_mpc/models```
* source bashrc file  

#### Launch Commands  
* ```roslaunch multi_robot_mpc multi_robot_nodes.launch``` launch node for each robot
* ```roslaunch multi_robot_mpc multi_robot_env.launch``` multirobot environment in gazebo  
* ```roslaunch multi_robot_mpc multi_robot_spawn.launch``` spawn robots in the environment 
  
[Youtube](https://www.youtube.com/watch?v=1mIJbgoKXOM)
