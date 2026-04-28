# RobotArmManipulation

This repository contains a robotics manipulation workspace focused on **robotic arm handling, grasping, and simulation**, developed mainly using **ROS**, **Gazebo**, and **GraspIt!**.  
It is structured as a complete ROS workspace and includes tools, scripts, notebooks, and Docker support to ensure reproducibility.

---

## 📁 Repository Structure

```bash
RobotArmManipulation/
├── docker/                 # Dockerfiles and container configuration
├── utils/                  # Auxiliary utilities
├── .devcontainer/          # VS Code devcontainer setup
├── manipulacion_ws/        # ROS workspace
│ ├── src/
│ │ ├── manipulacion_pkg/
│ │ │ ├── launch/           # ROS launch files
│ │ │ ├── urdf/             # Robot descriptions
│ │ │ ├── rviz/             # RViz configurations
│ │ │ ├── config/           # Configuration files
│ │ │ ├── objects_models/   # Object models for grasping
│ │ │ ├── graspit_grippers/ # Gripper models for GraspIt!
│ │ │ ├── images/           # Documentation images
│ │ │ ├── scripts/          # Python scripts and notebooks
│ │ │ └── README.md
│ │ ├── graspit_commander/
│ │ ├── graspit_interface/
│ │ └── gazebo-pkgs/
└── README.md
```

---

## 🤖 Main Features

- Robot arm manipulation in simulation
- Integration with **Gazebo**
- Grasp planning and evaluation using **GraspIt!**
- RViz visualization tools
- Python utilities for joint initialization and scene setup
- Educational Jupyter notebooks covering:
  - Robotic arm fundamentals
  - Coordinate systems and transformations
  - Grasp planning concepts
  - Gripper modeling and control

---

## 📦 `manipulacion_pkg/scripts`

This folder contains core scripts and notebooks:

### Python scripts
- `manipulacion_lib.py` – Common manipulation utilities
- `set_initial_joint_states.py` – Initialize robot joint states
- `set_joints_initial_position.py` – Set initial arm configuration
- `set_grippers_initial_joint_positions.py` – Initialize gripper joints
- `publish_obstacle_rviz.py` – Publish obstacles to RViz

### Jupyter notebooks
- `introduccion_al_brazo_robotico.ipynb`
- `guia_brazo_robotico_gazebo.ipynb`
- `guia_graspit.ipynb`
- `gripper_flotante_gazebo.ipynb`
- `sistemas_coordenadas_y_transformaciones.ipynb`

These notebooks are intended both as **learning material** and **practical experimentation tools**.

---

## 🐳 Docker Support

The project includes Dockerfiles to provide a fully reproducible environment with all dependencies installed.

### 🔨 Build the Docker image

From the **root of the project**:

```bash
docker build -t <image_name>:<tag> -f docker/<Dockerfile_name> .
```
> Important -> The build context is the current directory, so make sure to run this command from the root of the repository.

## 🔐 Login to GitHub Container Registry (GHCR)

To push the image to GitHub Container Registry, authenticate using a GitHub Personal Access Token (PAT):

```bash
export PAT=<your_personal_access_token>
echo $PAT | docker login ghcr.io -u <github_user> --password-stdin
```

## 🚀 Tag and Push the Image to GHCR
Tag the local image and push it to GitHub Container Registry:

```bash
docker tag <image_name>:<tag> ghcr.io/<github_user>/<image_name>:<tag>
docker push ghcr.io/<github_user>/<image_name>:<tag>
```

After this, the image will be available under Packages in your GitHub repository or organization.
