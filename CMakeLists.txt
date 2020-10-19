cmake_minimum_required(VERSION 2.8.3)
project(metrics_refbox_client)

find_package(catkin REQUIRED
  COMPONENTS
    roslint
    std_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    rospy
    std_msgs
)

### TESTS
roslint_python()

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)

  roslaunch_add_file_check(launch)
endif()

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(PROGRAMS
  scripts/metrics_refbox_client_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
