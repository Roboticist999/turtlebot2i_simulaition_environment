# turtlebot2i_simulaition_environment
0. You may need to copy turtlebot2i directory to your ~/ directory, and enter turtlebot2i directory, then catkin make
1. In order to use this environment, type roslaunch turtlebot2i_gazebo turtlebot2i_world.launch in terminal. In this case you will use the cafe world, if you want to use another world file, modify turtlebot2i_world.launch to use that file
2. There will be tons of errors and warnings, and GUI seem to die, but I think they don't matter as I'm running lots of nodes using this launch file and it may due to the limit of computation speed. The program will repeat operations with error. After Gazebo recovers, click the rectangle at bottom left of Gazebo to resume it.
3. Open another terminal and run roslaunch turtlebot_teleop keyboard.launch to use the keyboard to control the robot in Gazebo
4. You can disable Mapcloud in Rviz by unchecking it to accelerate graphical computation
5. You can save the map any time when you are mapping. You can save both rtabmap.db and .pgm file.
References:
http://wiki.ros.org/turtlebot_gazebo/Tutorials/indigo/Make%20a%20map%20and%20navigate%20with%20it
https://github.com/Interbotix/turtlebot2i/wiki/03:-Making-a-map-of-the-environment
