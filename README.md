# ROS2 Course Packages
This Git repository contains the packages of the ROS2 course provided in CS460 course entitled Introduction to Robotics. The repository includes the following packages:

* `ros2_essential_cpp`: This package contains essential concepts and examples of ROS2 using C++ language.
* `ros2_essential_python`: This package contains essential concepts and examples of ROS2 using Python language.
* `ros2_interfaces_cpp`: This package contains examples of defining and using custom ROS2 interfaces using C++ language.
* `ros2_motion_cpp`: This package contains examples of robot motion control using ROS2 and C++ language.
* `ros2_motion_python`: This package contains examples of robot motion control using ROS2 and Python language.

# Getting Started
To use these packages, you need to have ROS2 installed on your system. Follow the ROS2 installation guide to install ROS2 on your system.

Then, clone this Git repository to your local machine using the following command:

```bash
cd ~/ros2_ws/src
git clone https://github.com/riotu-lab/ros2_course_packages.git
cd ~/ros2_ws/
mv src/ros2_course_packages/* src/
rm -r src/ros2_course_packages
```
# Running Examples
To run the examples provided in each package, build the packages using the following command:

```bash
cd ~/ros2_ws
colcon build
```

Then, source the installed packages using the following command:

```bash
source install/setup.bash
```

You can now run any example provided in the packages using the ROS2 command ros2 run. For example, to run the talker node from ros2_essential_cpp package, use the following command:

```bash
ros2 run ros2_essential_cpp talker
```

# Maintainers
This repository is maintained by Prof. Anis Koubaa. If you have any questions or suggestions, please feel free to contact us.

# Contributing
If you find any issues with the packages or would like to contribute, please create a new issue or pull request on the GitHub repository.

# License
This project is licensed under Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.