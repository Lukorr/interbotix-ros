FROM ubuntu:20.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt update
RUN apt install -y catkin git g++ python3-rosdep2 python3-pip apt-file vim
RUN apt-file update
RUN ln -s /usr/bin/python3 /usr/bin/python

RUN mkdir -p /root/interbotix_ws/src
WORKDIR /root/interbotix_ws
RUN catkin_make
WORKDIR /root/interbotix_ws/src
RUN git clone https://github.com/Interbotix/interbotix_ros_arms.git
WORKDIR /root/interbotix_ws
RUN rosdep update
RUN rosdep install --from-paths src --ignore-src -r -y
RUN pip3 install modern_robotics ipython

RUN apt install -y joint-state-publisher librobot-state-publisher-dev python3-roslaunch librviz-dev rviz ros-std-msgs libactionlib-msgs-dev libtrajectory-msgs-dev ros-trajectory-msgs ros-actionlib-msgs libdynamic-reconfigure-config-init-mutex-dev libgazebo9-dev ros-shape-msgs libshape-msgs-dev ros-sensor-msgs libfcl-dev librandom-numbers-dev librandom-numbers0d libassimp-dev libboost-python-dev librviz4d python3-trajectory-msgs xvfb strace

# roslint
RUN git clone https://github.com/ros/roslint
WORKDIR /root/interbotix_ws/roslint
RUN cmake . && make && make install


# xacro
WORKDIR /root/interbotix_ws
RUN git clone https://github.com/ros/xacro.git
WORKDIR /root/interbotix_ws/xacro
RUN cmake . && make && make install
RUN pip3 install .


# ros_control
WORKDIR /root/interbotix_ws
RUN git clone https://github.com/ros-controls/ros_control.git
WORKDIR /root/interbotix_ws/ros_control/hardware_interface
RUN cmake . && make && make install

WORKDIR /root/interbotix_ws/ros_control/controller_interface
RUN cmake . && make && make install

WORKDIR /root/interbotix_ws/ros_control/controller_manager_msgs
RUN cmake . && make && make install

WORKDIR /root/interbotix_ws/ros_control/controller_manager
RUN cmake . && make && make install

WORKDIR /root/interbotix_ws/ros_control/transmission_interface
RUN cmake . && make && make install

WORKDIR /root/interbotix_ws/ros_control/joint_limits_interface
RUN cmake . && make && make install


# ros_controllers
WORKDIR /root/interbotix_ws
RUN git clone https://github.com/ros-controls/ros_controllers.git

WORKDIR /root/interbotix_ws/ros_controllers/effort_controllers
RUN git clone https://github.com/ros-controls/control_msgs.git
WORKDIR /root/interbotix_ws/ros_controllers/effort_controllers/control_msgs/control_msgs
RUN cmake . && make && make install

WORKDIR /root/interbotix_ws/ros_controllers/effort_controllers
RUN git clone https://github.com/ros-controls/control_toolbox.git

WORKDIR /root/interbotix_ws/ros_controllers/effort_controllers/control_toolbox
RUN git clone https://github.com/ros-controls/realtime_tools.git
WORKDIR /root/interbotix_ws/ros_controllers/effort_controllers/control_toolbox/realtime_tools
RUN cmake . && make && make install

WORKDIR /root/interbotix_ws/ros_controllers/effort_controllers/control_toolbox
RUN cmake . && make && make install

WORKDIR /root/interbotix_ws/ros_controllers/forward_command_controller
RUN cmake . && make && make install

WORKDIR /root/interbotix_ws/ros_controllers/effort_controllers
RUN cmake . && make && make install

WORKDIR /root/interbotix_ws/ros_controllers/joint_state_controller
RUN cmake . && make && make install

WORKDIR /root/interbotix_ws/ros_controllers/joint_trajectory_controller
RUN cmake . && make && make install


# gazebo
WORKDIR /root/interbotix_ws
RUN git clone https://github.com/ros-simulation/gazebo_ros_pkgs.git
WORKDIR /root/interbotix_ws/gazebo_ros_pkgs/gazebo_dev
RUN cmake . && make && make install

WORKDIR /root/interbotix_ws/gazebo_ros_pkgs/gazebo_msgs
RUN cmake . && make && make install

WORKDIR /root/interbotix_ws/gazebo_ros_pkgs/gazebo_ros
RUN cmake . && make && make install

WORKDIR /root/interbotix_ws/gazebo_ros_pkgs/gazebo_ros_control
RUN cmake . && make && make install


# moveit
WORKDIR /root/interbotix_ws
RUN git clone https://github.com/ros-planning/moveit.git
WORKDIR /root/interbotix_ws/moveit/moveit_commander
RUN cmake . && make && make install

WORKDIR /root/interbotix_ws/moveit/moveit_ros/planning_interface
RUN git clone https://github.com/ros-planning/moveit_msgs.git

WORKDIR /root/interbotix_ws/moveit/moveit_ros/planning_interface/moveit_msgs
RUN git clone https://github.com/wg-perception/object_recognition_msgs.git
WORKDIR /root/interbotix_ws/moveit/moveit_ros/planning_interface/moveit_msgs/object_recognition_msgs
RUN cmake . && make && make install

WORKDIR /root/interbotix_ws/moveit/moveit_ros/planning_interface/moveit_msgs
RUN git clone https://github.com/OctoMap/octomap_msgs.git
WORKDIR /root/interbotix_ws/moveit/moveit_ros/planning_interface/moveit_msgs/octomap_msgs
RUN cmake . && make && make install

WORKDIR /root/interbotix_ws/moveit/moveit_ros/planning_interface/moveit_msgs
RUN cmake . && make && make install

WORKDIR /root/interbotix_ws/moveit/moveit_core
RUN git clone https://github.com/ros/geometry2.git
WORKDIR /root/interbotix_ws/moveit/moveit_core/geometry2/tf2_eigen
RUN cmake . && make && make install

WORKDIR /root/interbotix_ws/moveit/moveit_core
RUN git clone https://github.com/ros/eigen_stl_containers.git
WORKDIR /root/interbotix_ws/moveit/moveit_core/eigen_stl_containers
RUN cmake . && make && make install

WORKDIR /root/interbotix_ws/moveit/moveit_core
RUN git clone https://github.com/ros/geometry.git
WORKDIR /root/interbotix_ws/moveit/moveit_core/geometry/eigen_conversions
RUN cmake . && make && make install

WORKDIR /root/interbotix_ws/moveit/moveit_core
RUN git clone https://github.com/ros-planning/geometric_shapes.git
WORKDIR /root/interbotix_ws/moveit/moveit_core/geometric_shapes
RUN cmake . && make && make install

WORKDIR /root/interbotix_ws/moveit/moveit_core
RUN git clone https://github.com/ros-planning/srdfdom.git
WORKDIR /root/interbotix_ws/moveit/moveit_core/srdfdom
RUN cmake . && make && make install

WORKDIR /root/interbotix_ws/moveit/moveit_core
RUN cmake . && make && make install

WORKDIR /root/interbotix_ws/moveit/moveit_ros/occupancy_map_monitor
RUN cmake . && make && make install

WORKDIR /root/interbotix_ws/moveit/moveit_ros/planning
RUN cmake . && make && make install

WORKDIR /root/interbotix_ws/moveit/moveit_ros/warehouse
RUN git clone https://github.com/ros-planning/warehouse_ros.git
WORKDIR /root/interbotix_ws/moveit/moveit_ros/warehouse/warehouse_ros
RUN cmake . && make && make install

WORKDIR /root/interbotix_ws/moveit/moveit_ros/warehouse
RUN cmake . && make && make install

WORKDIR /root/interbotix_ws/moveit/moveit_ros/move_group
RUN cmake . && make && make install

WORKDIR /root/interbotix_ws/moveit/moveit_ros/manipulation
RUN cmake . && make && make install

WORKDIR /root/interbotix_ws/moveit/moveit_ros/planning_interface
RUN git clone https://github.com/stack-of-tasks/eigenpy.git
WORKDIR /root/interbotix_ws/moveit/moveit_ros/planning_interface/eigenpy
RUN git submodule update --init
RUN cmake . && make && make install

WORKDIR /root/interbotix_ws/moveit/moveit_ros/planning_interface
RUN cmake . && make && make install


# visual tools
WORKDIR /root/interbotix_ws
RUN git clone https://github.com/ros-planning/moveit_visual_tools.git

WORKDIR /root/interbotix_ws/moveit_visual_tools
RUN git clone https://github.com/PickNikRobotics/rviz_visual_tools.git

WORKDIR /root/interbotix_ws/moveit_visual_tools/rviz_visual_tools
RUN git clone https://github.com/PickNikRobotics/graph_msgs.git
WORKDIR /root/interbotix_ws/moveit_visual_tools/rviz_visual_tools/graph_msgs
RUN cmake . && make && make install

WORKDIR /root/interbotix_ws/moveit_visual_tools/rviz_visual_tools
RUN cmake . && make && make install

WORKDIR /root/interbotix_ws/moveit_visual_tools
RUN cmake . && make && make install


# dynamixel
WORKDIR /root/interbotix_ws
RUN git clone https://github.com/ROBOTIS-GIT/dynamixel-workbench.git

WORKDIR /root/interbotix_ws/dynamixel-workbench/dynamixel_workbench_toolbox
RUN git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git
WORKDIR /root/interbotix_ws/dynamixel-workbench/dynamixel_workbench_toolbox/DynamixelSDK/ros
RUN cmake . && make && make install

WORKDIR /root/interbotix_ws/dynamixel-workbench/dynamixel_workbench_toolbox
RUN cmake . && make && make install


# rqt_plot
WORKDIR /root/interbotix_ws
RUN git clone https://github.com/ros-visualization/rqt_plot.git
WORKDIR /root/interbotix_ws/rqt_plot
RUN cmake . && make && make install


# joy
WORKDIR /root/interbotix_ws
RUN git clone https://github.com/ros-drivers/joystick_drivers.git

WORKDIR /root/interbotix_ws/joystick_drivers/joy
RUN git clone https://github.com/ros/diagnostics.git

WORKDIR /root/interbotix_ws/joystick_drivers/joy/diagnostics/diagnostic_updater
RUN git clone https://github.com/ros/common_msgs.git
WORKDIR /root/interbotix_ws/joystick_drivers/joy/diagnostics/diagnostic_updater/common_msgs/diagnostic_msgs
RUN cmake . && make && make install

WORKDIR /root/interbotix_ws/joystick_drivers/joy/diagnostics/diagnostic_updater
RUN cmake . && make && make install

WORKDIR /root/interbotix_ws/joystick_drivers/joy
RUN cmake . && make && make install


# move deps away
WORKDIR /root/interbotix_ws/deps
RUN mv ../dynamixel-workbench/ ../joystick_drivers/ ../moveit_visual_tools/ ../ros_controllers/ ../rqt_plot/ ../xacro/ ../gazebo_ros_pkgs/ ../moveit/ ../ros_control/ ../roslint/ ./


# ...and finally
WORKDIR /root/interbotix_ws
RUN catkin_make

RUN echo 'source /root/interbotix_ws/devel/setup.bash' >> /root/.bashrc

RUN ln -s /root/interbotix_ws/deps/xacro /root/interbotix_ws/src/
RUN ln -s /usr/local/bin/xacro /root/interbotix_ws/src/xacro/
RUN ln -s /root/interbotix_ws/deps/dynamixel-workbench/dynamixel_workbench_toolbox/DynamixelSDK/ros/devel/lib/libdynamixel_sdk.so /usr/lib

RUN sed -i 's/wx250s/vx250/g' /root/interbotix_ws/src/interbotix_ros_arms/interbotix_examples/python_demos/bartender.py
