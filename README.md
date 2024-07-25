# sirobo_ros

## how to run
``` bash
# in terminal 1
ros2 launch diffdrive_arduino diffbot.launch.py 
```

``` bash
# in terminal 2
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/diffbot_base_controller/cmd_vel_unstamped
```