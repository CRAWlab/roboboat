cmake_minimum_required(VERSION 2.8.3)
project(sysID)


find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  numpy
)

catkin_python_setup()
################################################
## Declare ROS messages, services and actions ##
################################################

#not sure about this section as I do generate messages but the roboboat control package also generates messages but doesn't have anything in this section

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )
###################################
## catkin specific configuration ##
###################################

catkin_package(
  LIBRARIES sysID
  CATKIN_DEPENDS rospy geometry_msgs numpy
)

###########
## Build ##
###########


include_directories(

  ${catkin_INCLUDE_DIRS}
)


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
install(PROGRAMS
   scripts/pure_surge1.py
   scripts/pure_sway1.py
   scripts/pure_yaw7.py
   scripts/zigzag_1_7.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )


#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_estop_comms.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
