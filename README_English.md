# turtlebot-walks

agent.py   A agent module. It mainly updates robot policies and then selects robot action on the certain state.

MDPy.py    MDP kits

world.py   A world module for parsing actions received from the agent module(agent.py). It will send the target position to the executor module, which is in the file test.py.
	
test.py    A communicating module with the turtlebot. It will send a goal position to the turtlebot and then it can receive the observation from the gazebo.   

replay.py  Used for timing and resetting the turtlebot position if timeout, so the turtlebot can be replayed in the gazebo. 

Operation：

    1、open some terminals，and input commands roslauncha turtlebot_gazebo turtlebot_world.launch，roslaunch turtlebot_gazebo amcl_demo.launch.roslaunch turtlebot_rviz_launchers view_navigation.launch separately in terminals. Before execute these commands, you may set some bash environment, like TURTLEBOT_GAZEBO_WORLD_FILE=/opt/ros/kinetic/share/turtlebot_gazebo/worlds/playground.world,  TURTLEBOT_GAZEBO_MAP_FILE=/opt/ros/kinetic/share/turtlebot_gazebo/maps/playground.yaml(you can change the file path according your files),
    
    2、start two terminals to start the experiment, one is for replay.py, and another is for agent.py.
    
Note: In this experiment, we use the scene file "playground.world" as a environment for the turtlebot which can be finded in the turtlebot_gazebo package.
