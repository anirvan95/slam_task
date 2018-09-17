# slam_task
Slam Task for kthsdv

1)Please change the filepath of data.mat in robotsense_publisher.py
2)clone the folder in catkin_ws/src
3)catkin_make
3)source ~/catkin_make/devel/setup.bash
4)roslaunch slam_kthsdv robot_localization.launch

Issues regarding package
If step 4 does not work, browse into the launch folder and roslaunch the file directly.

If $(find slam_kthsdv) line in the launch file does not work, please specify the full path of the config files

Also, included a bag file for reference of execution in my system. 

Regards

 
