cmake_minimum_required(VERSION 3.1.3)
project(roboboat_thrust_mapping)

find_package(catkin REQUIRED)

catkin_package()

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

catkin_install_python(PROGRAMS
  nodes/roboboat_thrust_mapping.py
  nodes/roboboat_raw_throttle_test.py
  nodes/roboboat_navio_thrust.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
