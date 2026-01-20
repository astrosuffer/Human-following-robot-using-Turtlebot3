# TurtleBot3 Human Following (ROS Noetic) — `turtlebot_human_tracking`

This repository contains a **ROS1 (Noetic) catkin workspace** and a ROS package that enables a **TurtleBot3** to **detect and follow a person** in Gazebo using a **MobileNetSSD** deep-learning detector (Caffe). 
The project is designed to be **reproducible from this repository alone**: it includes the launch files, the Python tracking node, the Gazebo worlds, and the trained network files.

---

## Table of Contents
- [Project Objective](#project-objective)
- [Repository Structure](#repository-structure)
- [System Architecture](#system-architecture)
- [Dependencies](#dependencies)
- [Build Instructions](#build-instructions)
- [Run Instructions](#run-instructions)
- [How to Verify / Debug](#how-to-verify--debug)
- [Parameters / Tuning](#parameters--tuning)
- [Limitations](#limitations)
- [Meaningful Commit History](#meaningful-commit-history)
- [License](#license)
- [Author](#author)

---

## Project Objective
Implement a **human-following behavior** for TurtleBot3 in simulation:
- Detect a person using **MobileNetSSD** (Caffe model).
- Estimate the target position from the camera image using the person **bounding box**.
- Command TurtleBot3 using velocity control (`/cmd_vel`) to follow the person.
- Provide Gazebo worlds for testing:
  - **with a person** (tracking + following)
  - **without people** (baseline / sanity check)

**Expected behavior**
- When a person appears in the robot camera view, the robot rotates to center the person and moves forward to follow.
- If the person disappears, the robot stops (or stops after a short timeout depending on implementation).

---

## Repository Structure
```text
.
├── .catkin_workspace
├── CMakeLists.txt -> /opt/ros/noetic/share/catkin/cmake/toplevel.cmake
├── README.md
├── run.txt
├── STRUCTURE.txt
└── turtlebot_human_tracking
    ├── CMakeLists.txt
    ├── launch
    │   ├── turtlebot3_follow_person.launch
    │   └── turtlebot3_map_without_people.launch
    ├── package.xml
    ├── scripts
    │   └── human_tracking.py
    ├── trained_network
    │   ├── MobileNetSSD_deploy.caffemodel
    │   └── MobileNetSSD_deploy.prototxt.txt
    └── worlds
        ├── world_without_people.world
        └── world_with_people.world

5 directories, 14 files


What each folder/file is for

Workspace files

.catkin_workspace: marks this repository root as a catkin workspace

CMakeLists.txt: catkin top-level build file (symlink/include to catkin)

README.md: full documentation (objective + build/run + deps + architecture)

run.txt: quick run notes (should be consistent with this README)

STRUCTURE.txt: repository tree output used for documentation

ROS package: turtlebot_human_tracking/

package.xml, CMakeLists.txt: ROS package definition/build configuration

launch/: launch files for simulation runs

scripts/human_tracking.py: main node (detection + follow control)

trained_network/: MobileNetSSD model files required for inference

worlds/: Gazebo worlds with and without people


System Architecture

This project implements a basic Perception → Control → Actuation pipeline using ROS topics.

High-level pipeline

Gazebo simulation publishes camera images (RGB stream).

human_tracking.py subscribes to the camera topic and runs MobileNetSSD inference.

From the detected person bounding box:

horizontal offset from image center → turning command (angular.z)

bounding box size (area/height) → forward command (linear.x, distance proxy)

The node publishes a geometry_msgs/Twist message to /cmd_vel.

TurtleBot3 base controller consumes /cmd_vel and moves the robot in Gazebo.


ROS dataflow (conceptual)

/camera/... (sensor_msgs/Image)
          |
          v
 [human_tracking.py: MobileNetSSD + controller]
          |
          v
      /cmd_vel (geometry_msgs/Twist)
          |
          v
    TurtleBot3 base (Gazebo)


Core ROS interfaces (typical)

Topic names can vary depending on TurtleBot3 simulation configuration and your launch files.

Subscribed

Camera image topic (e.g., /camera/rgb/image_raw)

Published

/cmd_vel (geometry_msgs/Twist)



Dependencies
OS + ROS

Ubuntu 20.04

ROS Noetic (ROS1)

TurtleBot3 simulation

Install the TurtleBot3 simulation stack (commonly used packages):

turtlebot3

turtlebot3_gazebo (or turtlebot3_simulations)

Python + OpenCV integration

human_tracking.py typically requires:

python3

rospy

sensor_msgs, geometry_msgs

OpenCV + ROS bridge:

cv_bridge

image_transport

python3-opencv (or system OpenCV)

Build Instructions

Run from the repository root (where .catkin_workspace exists).

1) Source ROS
source /opt/ros/noetic/setup.bash

2) Install dependencies with rosdep (recommended)
sudo apt-get update
sudo rosdep init || true
rosdep update
rosdep install --from-paths turtlebot_human_tracking --ignore-src -r -y

3) Build the workspace
catkin_make

4) Source the workspace overlay
source devel/setup.bash

5) Sanity check: package visible
rospack find turtlebot_human_tracking


If this prints a valid path, your build and overlay sourcing are correct.

Run Instructions
0) Set TurtleBot3 model (required)

In every new terminal:

export TURTLEBOT3_MODEL=burger   # or waffle / waffle_pi
source /opt/ros/noetic/setup.bash
source <REPO_ROOT>/devel/setup.bash

A) Full demo: world with a person + human following
roslaunch turtlebot_human_tracking turtlebot3_follow_person.launch

B) Baseline: world without people (sanity check)

Use this to verify the simulation environment without detection/following:

roslaunch turtlebot_human_tracking turtlebot3_map_without_people.launch

How to Verify / Debug

These commands help the examiner reproduce and troubleshoot quickly.

1) Check nodes are running
rosnode list

2) Check topics exist
rostopic list

3) Confirm camera is publishing

First find camera topics:

rostopic list | grep -i camera


Then check message rate (replace topic if needed):

rostopic hz /camera/rgb/image_raw

4) Confirm /cmd_vel is being published
rostopic echo /cmd_vel


If the robot is following, you should see changing values in:

linear.x

angular.z

5) Inspect the runtime connection graph
rqt_graph


You should see:

camera image topic → human_tracking.py node → /cmd_vel

6) Visual inspection (optional but recommended)
rviz


Add an Image display for the camera topic to confirm the detector input is valid.

Parameters / Tuning

The following behavior depends heavily on control constants:

min/max linear.x (forward speed clamp)

min/max angular.z (turn rate clamp)

“center tolerance” (how close the person must be to the image center)

“distance threshold” (based on bounding box size)

target-lost timeout (stop when no person detected)

If your human_tracking.py uses ROS parameters, document them here (names and defaults).
If the values are hardcoded in Python, document the exact constants in comments or move them into ROS params.

Limitations

Bounding box size is only a distance proxy (not true metric distance).

Sensitive to:

multiple people in view

occlusion

partial person visibility

camera blur / lighting changes

No obstacle avoidance unless separately implemented.

Deep learning inference speed depends on CPU/GPU performance.




