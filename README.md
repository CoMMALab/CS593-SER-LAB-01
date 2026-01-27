# panda_gz_moveit2

Franka Emika Panda manipulation with MoveIt 2 and Gazebo Harmonic simulation.

Updated to work with ROS 2 Jazzy and Gazebo Harmonic.

## Quick Start

### Build

```bash
.docker/build.bash
```

### Run

```bash
.docker/run.bash
```

### Test (inside container)

```bash
# MoveIt with fake controller (no simulation)
ros2 launch panda_moveit_config ex_fake_control.launch.py

# MoveIt with Gazebo simulation (includes tabletop scene with objects)
ros2 launch panda_moveit_config ex_gz_control.launch.py

# Just Gazebo visualization
ros2 launch panda_description view_gz.launch.py
```

## Tabletop Scene

The Gazebo simulation includes a tabletop scene with:
- A table in front of the robot
- A red box (0.15m cube) at (0.4, 0.2, 0.495)
- A blue cylinder (0.075m radius, 0.21m height) at (0.6, -0.2, 0.525)

Objects can be dragged in Gazebo and their positions are automatically synchronized to the MoveIt planning scene for collision avoidance.

### Moving Objects

Objects can be moved programmatically:

```bash
# Move red_box to a new position
ros2 run panda_moveit_config move_object.py red_box 0.5 0.0 0.495

# Move blue_cylinder
ros2 run panda_moveit_config move_object.py blue_cylinder 0.4 0.3 0.525
```

## Packages

- [**panda**](./panda) – Metapackage
- [**panda_description**](./panda_description) – URDF/SDF robot description
- [**panda_moveit_config**](./panda_moveit_config) – MoveIt 2 configuration

## Requirements

- Docker
- X11 (for GUI)
- NVIDIA Container Toolkit (for GPU acceleration, optional)

### Installing NVIDIA Container Toolkit

Required for Gazebo rendering with GPU acceleration:

```bash
# Add repository
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
  sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
  sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

# Install and configure
sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

### Running Without GPU

For systems without an NVIDIA GPU, use software rendering:

```bash
# Run with software rendering (Mesa llvmpipe)
.docker/run.bash -s
```

Alternatively, run Gazebo in headless mode (no GUI, faster):

```bash
# Inside container
ros2 launch panda_moveit_config ex_gz_control.launch.py headless:=true
```
