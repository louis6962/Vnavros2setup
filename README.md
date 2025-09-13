# Vnavros2setup

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
```

### 3) Build & source
```
colcon build --symlink-install
source install/setup.bash
```

### 4) Run the driver + bridge (defaults: VESC at /dev/ttyACM0)
```
ros2 launch launches vesc.launch.py
```

### 5) In a second terminal (build env again) run teleop
```
source /opt/ros/jazzy/setup.bash
cd ~/Vnavros2setup/workspaces/f1tenth_ws
source install/setup.bash
ros2 launch launches teleop.launch.py
```

# Install command for 435i (temp): 
sudo apt update
sudo apt install -y v4l-utils \
  ros-jazzy-librealsense2=2.56.4-1noble.20250722.140707 \
  ros-jazzy-realsense2-camera=4.56.4-1noble.20250814.083109 \
  ros-jazzy-realsense2-camera-msgs=4.56.4-1noble.20250806.110923

# permissions (safe to re-run)
sudo udevadm control --reload-rules && sudo udevadm trigger
sudo usermod -aG video $USER


# Launch command for 435i:
```
ros2 launch realsense2_camera rs_launch.py device_type:=d435i enable_color:=true enable_depth:=true enable_gyro:=true enable_accel:=true unite_imu_method:=linear_interpolation pointcloud.enable:=true align_depth:=true rgb_camera.profile:=1280x720x30 depth_module.profile:=1280x720x30
```



# Install commands for S2Pro Lidar (temp): 


# Launch command for S2Pro Lidar:
```

```
