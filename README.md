# myf1tenth_system

A collaborative project to develop and simulate autonomous racing algorithms for the [F1TENTH platform](https://f1tenth.org/).

## Collaborators:
- **Jaime García Arrojo**: [GitHub Profile](https://github.com/JaimeG-ELC)

---

## 🚀 Quick Start Guide (Beginner Friendly)

### Prerequisites
- **Operating System:** Ubuntu 20.04 (recommended) or compatible Linux/Windows with WSL2
- **Docker:**
- **NVIDIA GPU & Drivers:**
  - For CUDA support, install the latest [NVIDIA drivers](https://www.nvidia.com/Download/index.aspx) for your GPU
  - Install [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html) to enable GPU access in Docker

### 1. Clone the Repository
```bash
git clone https://github.com/JaimeG-ELC/coche_ws_repo.git
cd coche_ws_repo
```

### 2. Install Docker (if not already installed)
Run the provided script:
```bash
./install_docker.sh
```
Or follow the [official Docker installation guide](https://docs.docker.com/get-docker/).

### 3. Build the Docker Image
This will set up ROS2 Foxy and CUDA inside the container:
```bash
./build_container.sh
```

### 4. Launch the Container
```bash
./launch_container.sh
```
- The container will start with all dependencies (ROS2 Foxy, CUDA, and project packages) pre-installed.
- You will be dropped into a bash shell inside the container workspace.

### 5. Build the ROS2 Workspace (inside the container)
```bash
cd /root/coche_ws
colcon build
source /opt/ros/foxy/setup.bash
```

### 6. Run Your ROS2 Nodes
- Use standard ROS2 launch commands, e.g.:
```bash
ros2 launch <package_name> <launch_file.launch.py>
```

---

## Key Features

- ROS2-based development environment.
- CUDA-enabled Docker container for GPU-accelerated computation.
- Tools for building, running, and managing the system.
- Scripts for hardware setup and environment preparation.

### ROS2
- Communication protocol for easy data transfer and processing
- Ease of implementation and scalability
- Standardized system development

### Docker
- Containerizes the application to prevent OS and dependency issues
- Maintains good performance and efficient resource usage
- The Dockerfile creates the workspace and loads all dependencies

---

## Troubleshooting
- **NVIDIA GPU not detected in container?**
  - Make sure you installed the NVIDIA Container Toolkit and started Docker with `--gpus all` (see [NVIDIA docs](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/user-guide.html)).
  - In case you do not a gpu, the dockerfile has to be editted removing the CUDA installation and compiling the particle filter with the file <compile.sh>
- **Build errors?**
  - Double-check that you are running commands inside the container.
  - Ensure your host system meets the prerequisites above.

---

For more details, see the [official ROS2 Foxy documentation](https://docs.ros.org/en/foxy/index.html) and [NVIDIA CUDA documentation](https://docs.nvidia.com/cuda/).