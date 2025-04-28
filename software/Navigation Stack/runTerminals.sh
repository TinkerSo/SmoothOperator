#!/bin/bash

# Run smoothoperator config launch
lxterminal -e "bash -c 'cd ~/catkin_ws && source devel/setup.bash && roslaunch --wait smoothoperator smoothoperator_configuration.launch && exec bash'"

# Run move base
lxterminal -e "bash -c 'cd ~/catkin_ws && source devel/setup.bash && roslaunch --wait smoothoperator move_base.launch && exec bash'"

# Run roscore
lxterminal -e "bash -c 'cd ~/catkin_ws && source devel/setup.bash && roscore && exec bash'"

# Run robot_commands
lxterminal -e "bash -c 'cd ~/catkin_ws && source devel/setup.bash && rosrun robot_commands robot_commands.py && exec bash'"

# Run rviz
lxterminal -e "bash -c 'cd ~/catkin_ws && source devel/setup.bash && rosrun rviz rviz && exec bash'"

# Run node server
lxterminal -e "bash -c '/home/smoothoperator/.nvm/versions/node/v16.20.2/bin/node -v ; cd ~/Desktop/SmoothOperator/network/node-server && node DemoServer.js ; exec bash'"

# Run route_manager
lxterminal -e "bash -c 'cd ~/catkin_ws && source devel/setup.bash && rosrun route_manager route_manager.py && exec bash'"

# Run UI backend
lxterminal -e "bash -c 'cd ~/Desktop/SmoothOperator/network/node-server && exec bash'"

# Run UI frontend
lxterminal -e "bash -c 'cd ~/Desktop/SmoothOperator/PythonUI && exec bash'"
