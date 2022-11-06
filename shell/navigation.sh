#!/bin/bash
{
    gnome-terminal --tab "open_gazebo" -- bash -c "cd ~/cartographer_navigation/navigation_ws;source devel/setup.bash;roslaunch turtlebot3_gazebo turtlebot3_stage_4.launch;exec bash"
}&
sleep 4s

{
    gnome-terminal --tab "cartographer_localization" -- bash -c "cd ~/cartographer_navigation/cartographer_ws;source install_isolated/setup.bash;roslaunch cartographer_ros demo_backpack_2d_localization.launch;exec bash"
}&
sleep 4s

{
    gnome-terminal --tab "navigation" -- bash -c "cd ~/cartographer_navigation/navigation_ws;source devel/setup.bash;roslaunch move_base selfrobot_nav.launch;exec bash"
}
