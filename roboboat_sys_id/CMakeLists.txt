cmake_minimum_required(VERSION 2.8.3)
project(roboboat_sys_id)


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
  LIBRARIES roboboat_sys_id
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
   nodes/pure_surge0_4.py
   nodes/pure_surge0_5.py
   nodes/pure_surge0_6.py
   nodes/pure_surge0_75.py
   nodes/pure_surge0_8.py
   nodes/pure_surge1.py
   nodes/pure_surge1_25.py
   nodes/pure_surge1_5.py
   nodes/pure_surge2.py
   
   nodes/pure_sway0_4.py
   nodes/pure_sway0_5.py
   nodes/pure_sway0_6.py
   nodes/pure_sway0_75.py
   nodes/pure_sway0_8.py
   nodes/pure_sway1.py
   nodes/pure_sway1_25.py
   nodes/pure_sway1_5.py
   nodes/pure_sway2.py


   nodes/pure_yaw10.py
   nodes/pure_yaw15.py
   nodes/pure_yaw20.py
   nodes/pure_yaw30.py
   nodes/pure_yaw40.py
   nodes/pure_yaw60.py
   nodes/pure_yaw72.py
   nodes/pure_yaw80.py

   nodes/zigzag_0_5_60.py
   nodes/zigzag_1_60.py
   nodes/zigzag_1_90.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

 install(FILES
  launch/sys_id.launch
  # myfile2
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
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
