# turtlebot2i_simulaition_environment
1. If you want to configure the simulation environment by yourself. I created a log file named v1.0 turtlebot2i simulation environment set up.pdf. It might be helpful.
2. Before you download and use the simulation environment, follow the steps I marked as green in Full Build Instructions · Interbotix_turtlebot2i Wiki.pdf. They are basically ROS and ROS packages installation and ~/.bashrc modification, etc. Don't do steps I marked as yellow, as those packages have been downloaded in the simulation environment.
3. Install gazebo_ros_pkgs and gazebo_ros_control package using

sudo apt-get install ros-kinetic-gazebo-ros-pkgs ros-kinetic-gazebo-ros-control

4. You may need to copy turtlebot2i directory to your ~/ directory, and enter turtlebot2i directory, then "catkin make"

5.1 In order to use this environment, type "roslaunch turtlebot2i_gazebo turtlebot2i_world2.launch" in terminal. In this case you will use the cafe world, if you want to use another world file, modify turtlebot2i_world.launch to use that file

5.2 Run "roslaunch turtlebot2i_bringup rviz.launch" in a new terminal to start Rviz. If the Rviz model is not correctly matching the transforms, change the Rviz global Fixed Frame to be 'odom'.

5.3 Run "roslaunch turtlebot2i_bringup rtabmap.launch args:=--delete_db_on_start" in a new terminal to start building a new map. My rtablap.launch still has issues, which is camera image turns red as demonstrated at the end of "v1.2 turtlebot2i simulation environment set up.pdf"

6. There will be tons of errors and warnings, and GUI seem to die, but I think they don't matter as I'm running lots of nodes using this launch file and it may due to the limit of computation speed. The program will repeat operations with error. After Gazebo recovers, click the rectangle at bottom left of Gazebo to resume it.
7. Open another terminal and run roslaunch turtlebot_teleop keyboard.launch to use the keyboard to control the robot in Gazebo
8. You can disable Mapcloud in Rviz by unchecking it to accelerate graphical computation
9. You can save the map any time when you are mapping. You can save both rtabmap.db and .pgm file.
References:
http://wiki.ros.org/turtlebot_gazebo/Tutorials/indigo/Make%20a%20map%20and%20navigate%20with%20it
https://github.com/Interbotix/turtlebot2i/wiki/03:-Making-a-map-of-the-environment
