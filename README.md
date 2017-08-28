# turtlebot-walks

agent.py   主要用于机器人的策略更新、动作选择等

MDPy.py    是MDP kits

world.py   是世界模块，用于解析agent模块发来的动作，并返回给agent模块执行情况，即用来与机器人通信，同时又把目标位置等信息发送给真正执行的模块，该模块于test.py中。
	
test.py    是与turtlebot通信的模块，用来给turtlebot发送目标位置，同时接收执行情况，然后把情况返回给世界模块。

replay.py  用于计时、恢复turtlebot原始位置等功能，使turtlebot循环在gazebo中运动。

操作方式：

    1、首先需要在几个终端，分别执行roslauncha turtlebot_gazebo turtlebot_world.launch，roslaunch turtlebot_gazebo amcl_demo.launch.roslaunch turtlebot_rviz_launchers view_navigation.launch.在执行之前，可能需要设置几个环境变量，如TURTLEBOT_GAZEBO_WORLD_FILE=/opt/ros/kinetic/share/turtlebot_gazebo/worlds/playground.world,  TURTLEBOT_GAZEBO_MAP_FILE=/opt/ros/kinetic/share/turtlebot_gazebo/maps/playground.yaml,
    
    2、然后，启动两个终端，一个执行replay.py文件，另一个执行agent.py文件，即可进行仿真。
    
注意：这里使用的是turtlebot_gazebo中的playground.world的场景。
