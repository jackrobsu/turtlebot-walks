# turtlebot-walks

agent.py   主要用于机器人的策略更新、动作选择等

MDPy.py    是MDP kits

world.py   是世界模块，用于解析agent模块发来的动作，并返回给agent模块执行情况，即用来与机器人通信，同时又把目标位置等信息发送给真正执行的模块，该模块       
           位于test.py中。
	
test.py    是与turtlebot通信的模块，用来给turtlebot发送目标位置，同时接收执行情况，然后把情况返回给世界模块。

replay.py  用于计时、恢复turtlebot原始位置等功能，使turtlebot循环在gazebo中运动。

注意：这里使用的是turtlebot_gazebo中的playground.world的场景。
