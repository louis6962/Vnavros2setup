# Vnavros2setup

## Instructions to launch teleop & vesc:

### (1) Go to the workspace
```cd ~/Vnavros2setup/workspaces/f1tenth_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash 2>/dev/null || true
```

### (2) Set device variables (per shell)
```export VESC_DEV=/dev/serial/by-id/usb-STMicroelectronics_ChibiOS_RT_Virtual_COM_Port_304-if00
export JOY_DEV=/dev/input/js0
```

### 3) Build then re-source
```colcon build --symlink-install --packages-up-to launches vesc_driver vesc vesc_ackermann f1tenth_teleop
source install/setup.bash
```

### 4) Launch using the variables
```
ros2 launch launches driver.launch.py vesc_port:=$VESC_DEV joy_dev:=$JOY_DEV
```

## Launch command for 435i:
```
ros2 launch realsense2_camera rs_launch.py device_type:=d435i enable_color:=true enable_depth:=true enable_gyro:=true enable_accel:=true unite_imu_method:=linear_interpolation pointcloud.enable:=true align_depth:=true rgb_camera.profile:=1280x720x30 depth_module.profile:=1280x720x30
```




## Launch command for S2Pro Lidar:
```

```
