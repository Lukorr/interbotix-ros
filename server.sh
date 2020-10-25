#!/bin/bash
xvfb-run roslaunch interbotix_sdk arm_run.launch robot_name:=$1 use_time_based_profile:=true gripper_operating_mode:=pwm
