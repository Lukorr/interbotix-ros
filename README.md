# Interbotix ROS
Image with ROS and the Interbotix sdk, since there are no official images for
the latter and the official ROS images don't support all dependencies.

Run server with `xvfb-run roslaunch interbotix_sdk arm_run.launch
robot_name:=vx250 use_time_based_profile:=true gripper_operating_mode:=pwm`
(replace vx250 with your arm), and run your code afterwards.
