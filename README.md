# panda_gz_moveit2

Franka Emika Panda manipulation with MoveIt 2 and Gazebo simulation.

Updated to work with ROS2 Jazzy.

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

# MoveIt with Gazebo simulation
ros2 launch panda_moveit_config ex_gz_control.launch.py

# Just Gazebo visualization
ros2 launch panda_description view_gz.launch.py
```

## Packages

- [**panda**](./panda) – Metapackage
- [**panda_description**](./panda_description) – URDF/SDF robot description
- [**panda_moveit_config**](./panda_moveit_config) – MoveIt 2 configuration

## Requirements

- Docker
- X11 (for GUI)
- NVIDIA Container Toolkit (for GPU acceleration)

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

Without GPU support, use software rendering (slower):
```bash
.docker/run.bash -e LIBGL_ALWAYS_SOFTWARE=1
```
