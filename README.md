# ToastBot
## Thermal Optimization and Actuation of Sliced Toast Robot (T.O.A.S.T Bot)
Authors: Sharwin Patil, Grayson Snyder, Asa Rogers, and Tony Shilati

## Overview
Toastbot is a ros2 package that relies on the moveitapi package to command a Franka Emika Panda robot pick up a piece of bread, toast it, and place it on a plate. The package uses April tags to track objects in the physical environment of the robot and to track when object states change.

This repository depends on the [moveit API](https://github.com/ME495-EmbeddedSystems/moveitapi-group5) developed by the same authors. To maintain this project and the API separately, it is a submodule in this repo and must be updated to include it.



## Quickstart

When first cloning the repo:
```bash
# Ensure you're cloning in your ws/ directory
git clone --recurse-submodules git@github.com:snydergi/ToastBot.git
```

To update the submodules when there are changes made to their repositories run:
```bash
git submodule init # If you're missing submodule
git submodule update
```

## Calling Services

## setScene
```bash
ros2 service call /buildScene
```
Creates the scene with the objects in simulation so planned paths account for them.

## breadToToaster
```bash
ros2 service call /breadToToaster
```
Bread from Loaf Tray to Toaster Slot 0

## actuateLever
```bash
ros2 service call /actuateLever
```
Press the Lever on the Toaster until it clicks.

## goHome
```bash
ros2 service call /gohome
```
Sends the Franka to the Home Position

## toastToPlate
```bash
ros2 service call /toastToPlate
```
Grabs from toaster slot 0 and plates it

## openGripper
```bash
ros2 service call /openGripper
```
Opens the gripper

## closeGripper
```bash
ros2 service call /closeGripper
```
Closes the gripper

## initiateToasting
```bash
ros2 service call /initiateToasting
```
1. Opens the gripper
2. Go to Loaf Tray
3. Close Gripper
4. Lift Bread out of Loaf Tray
5. Go to Home Position
6. Move Bread directly over Toaster Slot 0
7. Move Bread into slot
8. Open the gripper
9. Go to Home Position
10. Waits for lever pose to exist
11. Move above Lever
12. Press Lever
13. Move up above lever
14. Go to Home Position

## Resources
[April Tag Generator](https://chaitanyantr.github.io/apriltag.html)

[YOLO ROS Support](https://github.com/mgonzs13/yolo_ros)


# Camera (Realsense)
## Dependencies
1. Apriltags
    - install by running the following:
    `sudo apt install ros-jazzy-apriltag-ros`

To launch
`colcon clean workspace -y && colcon build && source install/setup.bash && ros2 launch realsense camera.launch.py`
