cmake_minimum_required(VERSION 2.8.3)
project(tbot2_laser_rw)
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  kobuki_msgs
  geometry_msgs
  nav_msgs
)

catkin_python_setup()
catkin_package()
