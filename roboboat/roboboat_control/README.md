# roboboat_control Package

This package uses the [ACADO toolkit](https://acado.github.io/) to generate a Model Predictive Control for [UL Lafayette](https://www.louisiana.edu)'s RoboBoat.

The RoboBoat depends on [ROS](http://wiki.ros.org/), and some modifications have to be done to the default ROS packages for this to work. 

## Dependencies:

### Build
1. [catkin](wiki.ros.org/catkin?distro=melodic)
2. [roscpp](http://wiki.ros.org/roscpp)
3. [rospy](wiki.ros.org/rospy/)
4. [tf](http://wiki.ros.org/tf)
5. [nav_msgs](http://wiki.ros.org/nav_msgs)
6. [geometry_msgs](http://wiki.ros.org/geometry_msgs)
7. [std_msgs](http://wiki.ros.org/std_msgs)
8. [roslint](http://wiki.ros.org/roslint)
9. roboboat_msgs, contained within this workspace
10. [ACADO](http://wiki.ros.org/acado)

 [The normal installation](http://acado.github.io/using_cmake_unix_common.html) requires generating a `build` directory within the files generated by `git clone`, but the installation that results from `sudo apt-get install ros-melodic-acado` installs pointers to `/opt/ros/melodic/include/` from `/opt/ros/melodic/share`. The `build` folder created from the normal installation creates files with the proper system structure, but this does not match that of ROS.

 The following commands need to be performed in order to install ACADO for ROS and have it properly read (sudo required), *after* ROS has been successfully installed on your system:

1. `sudo apt-get install ros-melodic-acado`
2. `cd`
3. `cd /opt/ros/melodic/include/acado`
4. `sudo mkdir -p acado`
5. `sudo cp -r -d bindings/ /opt/ros/melodic/include/acado/acado/`
6. `sudo cp -r -d clock/ /opt/ros/melodic/include/acado/acado/`
7. `sudo cp -r -d code_generation/ /opt/ros/melodic/include/acado/acado/`
8. `sudo cp -r -d conic_program/ /opt/ros/melodic/include/acado/acado/`
9. `sudo cp -r -d conic_solver/ /opt/ros/melodic/include/acado/acado/`
10. `sudo cp -r -d constraint/ /opt/ros/melodic/include/acado/acado/`
11. `sudo cp -r -d control_law/ /opt/ros/melodic/include/acado/acado/`
12. `sudo cp -r -d controller/ /opt/ros/melodic/include/acado/acado/`
13. `sudo cp -r -d curve/ /opt/ros/melodic/include/acado/acado/`
14. `sudo cp -r -d dynamic_discretization/ /opt/ros/melodic/include/acado/acado/`
15. `sudo cp -r -d dynamic_system/ /opt/ros/melodic/include/acado/acado/`
16. `sudo cp -r -d estimator/ /opt/ros/melodic/include/acado/acado/`
17. `sudo cp -r -d external_packages/ /opt/ros/melodic/include/acado/acado/`
18. `sudo cp -r -d function/ /opt/ros/melodic/include/acado/acado/`
19. `sudo cp -r -d integrator/ /opt/ros/melodic/include/acado/acado/`
20. `sudo cp -r -d matrix_vector/ /opt/ros/melodic/include/acado/acado/`
21. `sudo cp -r -d nlp_derivative_approximation/ /opt/ros/melodic/include/acado/acado/`
22. `sudo cp -r -d nlp_solver/ /opt/ros/melodic/include/acado/acado/`
23. `sudo cp -r -d noise/ /opt/ros/melodic/include/acado/acado/`
24. `sudo cp -r -d objective/ /opt/ros/melodic/include/acado/acado/`
25. `sudo cp -r -d ocp/ /opt/ros/melodic/include/acado/acado/`
26. `sudo cp -r -d optimization_algorithm/ /opt/ros/melodic/include/acado/acado/`
27. `sudo cp -r -d process/ /opt/ros/melodic/include/acado/acado/`
28. `sudo cp -r -d reference_trajectory/ /opt/ros/melodic/include/acado/acado/`
29. `sudo cp -r -d set_arithmetics/ /opt/ros/melodic/include/acado/acado/`
30. `sudo cp -r -d simulation_environment/ /opt/ros/melodic/include/acado/acado/`
31. `sudo cp -r -d sparse_solver/ /opt/ros/melodic/include/acado/acado/`
32. `sudo cp -r -d symbolic_expression/ /opt/ros/melodic/include/acado/acado/`
33. `sudo cp -r -d symbolic_operator/ /opt/ros/melodic/include/acado/acado/`
34. `sudo cp -r -d transfer_device/ /opt/ros/melodic/include/acado/acado/`
35. `sudo cp -r -d user_interaction/ /opt/ros/melodic/include/acado/acado/`
36. `sudo cp -r -d utils/ /opt/ros/melodic/include/acado/acado/`
37. `sudo cp -r -d validated_integrator/ /opt/ros/melodic/include/acado/acado/`
38. `sudo cp -r -d variables_grid/ /opt/ros/melodic/include/acado/acado/`

Using `cp` instead of something else will cause other packages that might depend on `ACADO` using the ROS configuration provided by `sudo apt-get install ros-melodic-acado` in another manner to still properly function.

### Runtime
1. [roscpp](http://wiki.ros.org/roscpp)
2. [rospy](wiki.ros.org/rospy/)
3. [tf](http://wiki.ros.org/tf)
4. [nav_msgs](http://wiki.ros.org/nav_msgs)
5. [geometry_msgs](http://wiki.ros.org/geometry_msgs)
6. [std_msgs](http://wiki.ros.org/std_msgs)
7. [roslint](http://wiki.ros.org/roslint)
8. roboboat_msgs, contained within this workspace
9. [ACADO](http://wiki.ros.org/acado)


## Scripts
1. `controller.cpp`: MPC controller
2. `vel_cov.py`: Velocity Covariance Publisher, subscribes to `navsat/vel` and `navsat/fix`. Created by [this project](https://clearpathrobotics.com/blog/2019/01/heron-usv-gets-a-new-simulator/).
3. `control_allocation.py`: Maps thrusts from `/cmd_vel` to Holonomic Thruster Configuration used by UL Lafayette's RoboBoat.