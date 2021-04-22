### Multi Robot MPC  
Prerequisite Package: [Turtlebot3](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/)    
Change gazebo model path by adding this line in ~/.bashrc  
*```sudo gedit ~/.bashrc```  
*add ```export GAZEBO_MODEL_PATH=/home/user_name/workspace_name/src/multi_robot_mpc/models```  
Commands: 
* ```roslaunch multi_robot_mpc multi_robot_nodes.launch``` launch DMPC node for each robot
* ```roslaunch multi_robot_mpc multi_robot_env.launch``` multirobot environment in gazebo
