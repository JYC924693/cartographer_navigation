[Maze Explorer]

Robot gunakan laser data untuk explore maze ini. Robot akan sentiasa ikut dinding / wall yang di sebelah kirinya. 

Kalau robot bergerak terlalu laju / perlahan, boleh ubah nilai 
command.angular.z dan command.linear.x 

--------------------------------------------------------

Sebelum boleh run, perlu copy dan jadikan fail ini 'executable':
1. Copy dan paste fail maze_explorer.py dalam folder berikut:
    /home/user/catkin_ws/src/fira_maze/script
    
2. Buka terminal, taip:
    $ cd /home/arif/catkin_ws/src/fira_maze/script
    
3. Dalam terminal yang sama, jadikan executable:
    $ chmod +x maze_explorer.py
    
--------------------------------------------------------

To run on real robot:

Terminal 1:
$ roscore

Terminal 2: (turn on robot node)
$ ssh pi@raspberrypi.local sudo date -s$(date -Ins)
$ ssh pi@raspberrypi.local
$ roslaunch turtlebot3_bringup turtlebot3_robot.launch

Terminal 3: (turn on laptop remote control node)
$ roslaunch turtlebot3_bringup turtlebot3_remote.launch

Terminal 4: (turn on maze explorer node)
$ rosrun fira_maze maze_explorer.py

--------------------------------------------------------

To run on simulation:

Terminal 1:
$ roscore

Terminal 2: (turn on Gazebo)
$ roslaunch fira_maze maze_1_world.launch

Terminal 3: (turn on maze explorer node)
$ rosrun fira_maze maze_explorer.py

--------------------------------------------------------





