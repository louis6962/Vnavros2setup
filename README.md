# Vnavros2setup
https://github.com/vedderb/vesc_tool/tree/master/res

github for Vesc Firmwares

https://github.com/vedderb/bldc

# Install command for Vesc + firmware (temp): 

### 1) Go to the repo
```cd ~/Vnavros2setup```

### 2) Ensure flatpak + flathub
```sudo apt update && sudo apt install -y flatpak
flatpak remote-add --if-not-exists flathub https://flathub.org/repo/flathub.flatpakrepo```
```

### 3) Try installing the exact pinned commit (preferred)
```if [ -f external/vesc_tool/vesc_tool.flatpak.commit ]; then
  COMMIT=$(cat external/vesc_tool/vesc_tool.flatpak.commit)
  echo "Installing VESC Tool at commit: $COMMIT"
  flatpak install -y --noninteractive flathub com.vesc_project.VescTool//stable --commit="$COMMIT" || FALLBACK=1
else
  FALLBACK=1
fi
```

### 4) If commit install didn’t run, try your flatpakref from the repo
```if [ "${FALLBACK:-0}" = "1" ]; then
  if [ -f external/vesc_tool/vesc_tool.flatpak.info ]; then
    cp external/vesc_tool/vesc_tool.flatpak.info /tmp/vesc_tool.flatpakref
    flatpak install -y /tmp/vesc_tool.flatpakref || SECOND_FALLBACK=1
  else
    SECOND_FALLBACK=1
  fi
fi
```

### 5) Final fallback (only if neither repo file worked)
```if [ "${SECOND_FALLBACK:-0}" = "1" ]; then
  flatpak install -y flathub com.vesc_project.VescTool
fi
```

### 6) Serial permissions (once per user session)
```sudo usermod -aG dialout $USER
newgrp dialout
```

### 7) Run it
```flatpak run com.vesc_project.VescTool```

# If requiring motor recalibration

### 1) Go into vesctool (flatpak run or ./ depending on car)

### 2) Turn car upside down and do foc detection with small inrunner (200g) with full vesc battery

### 3) Set MotorSettings/Advanced/Maximum_input_voltage to 6.2V 

### 4) Set AppSettings/General/EnableServoOutput to TRUE (very important cannot turn car without this)

### 5) Write app config and motor settings to VESC

# If requiring bluetooth repairing
### 1) Enter bluetoothctl in terminal

### 2) Press the center bottom button and the share button at same time for around 5 seconds until lighbar rapidly flashes

### 3) Enter "scan on" in bluetoothctl and find the wireless controller. Example:     (Note that there may be multiple wireless controllers in room you may need to trial and error and unpair) [Additionally may need to repeat step 2 if pairing mode on controller stops early]
~~~
[bluetooth]# [CHG] Device BB:8E:41:F5:5D:C7 Name: Wireless Controller
[bluetooth]# [CHG] Device BB:8E:41:F5:5D:C7 Alias: Wireless Controller
~~~

### 4) Pair/trust/&connect with wireless controller. Example:
~~~
[bluetooth]# pair BB:8E:41:F5:5D:C7
[bluetooth]# trust BB:8E:41:F5:5D:C7
[bluetooth]# connect BB:8E:41:F5:5D:C7
~~~



# If requiring joy or vesc yaml reconfiguration (NOTE THAT A GIT PULL MAY OVERRIDE THESE SETTINGS AND NEED TO BE REDONE)

### 1) Go to teleop_twist_joy.yaml
~~~
cd ~/Vnavros2setup/workspaces/f1tenth_ws/src/f1tenth_teleop/config
nano teleop_twist_joy.yaml
~~~

### 2) Edit the needed axis and button using ros2 topic echo /joy and the number down for axis for controller while teleop is posting and and the enable button being l1. (Note index is 0, so the first axis that appears is axis 0 and the first button that appears is also button 0) 

### 3) Exit out and save with ctrl+x



# Instructions to launch teleop & vesc:

### 1) Go to Workspace
```
cd ~/Vnavros2setup/workspaces/f1tenth_ws
```

### 2) Install deps
```
source /opt/ros/jazzy/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y || true
sudo apt install -y ros-jazzy-asio-cmake-module ros-jazzy-io-context ros-jazzy-serial-driver
```

### 3) Build & source
```
colcon build --symlink-install
source install/setup.bash
```
### 3) Find VESC and Joystick ports
```
ls -l /dev/serial/by-id/
ls -l /dev/input/js*
# Vesc example result:
# usb-STMicroelectronics_ChibiOS_RT_Virtual_COM_Port_304-if00 -> ../../ttyACM0

# Joystick example result:
# /dev/input/js0
```

### 4) Run the driver + bridge (defaults: VESC at /dev/ttyACM0) 
```
export VESC_DEV=/dev/ttyACM0
ros2 launch launches vesc.launch.py vesc_port:=$VESC_DEV
```

### 5) In a second terminal (build env again) run teleop (defaults: Joy at /dev/input/js0) 
```
source /opt/ros/jazzy/setup.bash
cd ~/Vnavros2setup/workspaces/f1tenth_ws
source install/setup.bash
export JOY_DEV=/dev/input/js0
ros2 launch launches teleop.launch.py joy_dev:=$JOY_DEV
```

# Install command for 435i (temp): 
```
sudo apt update
sudo apt install -y v4l-utils \
  ros-jazzy-librealsense2=2.56.4-1noble.20250722.140707 \
  ros-jazzy-realsense2-camera=4.56.4-1noble.20250814.083109 \
  ros-jazzy-realsense2-camera-msgs=4.56.4-1noble.20250806.110923
```

# permissions (safe to re-run)
sudo udevadm control --reload-rules && sudo udevadm trigger
sudo usermod -aG video $USER


# Launch command for 435i:
```
ros2 launch realsense2_camera rs_launch.py device_type:=d435i enable_color:=true enable_depth:=true pointcloud.enable:=true align_depth:=true rgb_camera.profile:=1280x720x30 depth_module.profile:=1280x720x30
```



# Install commands for S2Pro Lidar (temp): 


# Launch command for S2Pro Lidar:
```

```
