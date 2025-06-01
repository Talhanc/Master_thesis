# Master_thesis
This is an Error-state Kalman filter for state estimation, and Transformed Unscented Kalman filter for recusive system identification for AUVs. This is part of my Master thesis.  



packages that are of interest in regards to the Master thesis is the following:

#### `navigation/`
Contains implementations related to navigation and state estimation.
- **Error-State Kalman Filter (ESKF)**.
- **Transformed Unscented Kalman Filter (TUKF)** for **Recursice System Identification**

#### `control/`
Includes controllers developed for system control.
- **Dynamic Positioning (DP) Adaptive Backstepping Controller**.


## How to setup 

This project requires:

- **Ubuntu 22.04**
- **[ROS 2 Humble]([https://docs.ros.org/en/humble/index.html])**

### 1. Create a ROS 2 workspace

```bash
mkdir  ~/ros2_ws
cd ~/ros2_ws
mkdir  src
cd src
```

### 2. Clone this repository into the `src` folder
```bash
git clone https://github.com/yourusername/master-thesis.git
```

### 3. Build the workspace
```bash
colcon build
```

### 4. Source the workspace
```bash
source install/setup.bash
```

### 5. Launch the ROS 2 nodes
```bash
ros2 launch eskf eskf.launch.py
ros2 launch tukf_rsi tukf_rsi.launch.py
```
