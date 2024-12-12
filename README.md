# ToastBot
## Thermal Optimization and Actuation of Sliced Toast Robot (T.O.A.S.T Bot)
Authors: Sharwin Patil, Asa Rogers, Tony Shilati, and Grayson Snyder

## Overview
Toastbot is a ros2 package that relies on the moveitapi package to command a Franka Emika Panda robot pick up a piece of bread, toast it, and place it on a plate. The package uses April tags to track objects in the physical environment of the robot and to track when object states change.

This repository depends on the [moveit API](https://github.com/ME495-EmbeddedSystems/moveitapi-group5) developed by the same authors. To maintain this project and the API separately, it is a submodule in this repo and must be updated to include it.



## Getting Setup

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

## Quickstart

To run the package and make toast, do the following commands:
```bash
colcon build
source install/setup.bash
ros2 launch toast make_toast.launch.py
ros2 service call /initiateToasting std_srvs/srv/Empty
```

## Video Demo
A video of the toast_bot in action:

https://private-user-images.githubusercontent.com/182719158/395237291-1bb0c27d-702d-469f-8133-5667d3e67510.webm?jwt=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJnaXRodWIuY29tIiwiYXVkIjoicmF3LmdpdGh1YnVzZXJjb250ZW50LmNvbSIsImtleSI6ImtleTUiLCJleHAiOjE3MzQwMzkyNjYsIm5iZiI6MTczNDAzODk2NiwicGF0aCI6Ii8xODI3MTkxNTgvMzk1MjM3MjkxLTFiYjBjMjdkLTcwMmQtNDY5Zi04MTMzLTU2NjdkM2U2NzUxMC53ZWJtP1gtQW16LUFsZ29yaXRobT1BV1M0LUhNQUMtU0hBMjU2JlgtQW16LUNyZWRlbnRpYWw9QUtJQVZDT0RZTFNBNTNQUUs0WkElMkYyMDI0MTIxMiUyRnVzLWVhc3QtMSUyRnMzJTJGYXdzNF9yZXF1ZXN0JlgtQW16LURhdGU9MjAyNDEyMTJUMjEyOTI2WiZYLUFtei1FeHBpcmVzPTMwMCZYLUFtei1TaWduYXR1cmU9ZGFmYmVjYjNkNDZjN2RjNGIwODdlZGQ2NDFmNmEyNTgxMWJhM2U3NmVhMTg2ZjczMDEyMGEyZDVkZDI2MmUxMCZYLUFtei1TaWduZWRIZWFkZXJzPWhvc3QifQ.jZpCD-IwkWvGHg0n3367vFth5oG6CU3h9bf9w6elZ5M

A screen-recording of the rviz configuration:

https://private-user-images.githubusercontent.com/83177842/395339003-dfdb436f-f03d-4812-82c8-f2e3e6608080.webm?jwt=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJnaXRodWIuY29tIiwiYXVkIjoicmF3LmdpdGh1YnVzZXJjb250ZW50LmNvbSIsImtleSI6ImtleTUiLCJleHAiOjE3MzQwNDA0OTAsIm5iZiI6MTczNDA0MDE5MCwicGF0aCI6Ii84MzE3Nzg0Mi8zOTUzMzkwMDMtZGZkYjQzNmYtZjAzZC00ODEyLTgyYzgtZjJlM2U2NjA4MDgwLndlYm0_WC1BbXotQWxnb3JpdGhtPUFXUzQtSE1BQy1TSEEyNTYmWC1BbXotQ3JlZGVudGlhbD1BS0lBVkNPRFlMU0E1M1BRSzRaQSUyRjIwMjQxMjEyJTJGdXMtZWFzdC0xJTJGczMlMkZhd3M0X3JlcXVlc3QmWC1BbXotRGF0ZT0yMDI0MTIxMlQyMTQ5NTBaJlgtQW16LUV4cGlyZXM9MzAwJlgtQW16LVNpZ25hdHVyZT0wYzUzNDEwMDk1MDVkZjJkOGZjN2EwZTNjMGE4OWJkMzdiZDk3YmUzMjk0YmUyOWY4N2JhNzIxMDY5MTFhMmUxJlgtQW16LVNpZ25lZEhlYWRlcnM9aG9zdCJ9.nC0CZp17gjt21W-p0oWIMdJtwAOh9CTTAV_Hn57gyTg

## Nodes and Launchfiles
### toast_bot
The toast_bot node is the main node which contains all the services necessary to accomplish the goal outlined in the overview.

### transform_auditor
The transform_auditor is used to constantly update and publish the poses of the scenes April Tags on topics used by subscribers in toast_bot.

### make_toast.launch.py
This launch file launches all necessary nodes and rviz2 to make the robot ready to make toast.

## System Architecture
The package uses RealSense cameras for our computer vision, utilizing the ROS April Tag package. With transforms made for each tag, the toast_bot contains services necessary to perform each action in the process of making toast, as well as a service that combines each intermediate step. A multi-thread executor is used in the toast_bot node such that once the toast completes and the toaster raises, the executor executes the remaining functions autonomously to allow for only one service to be called for the whole process.

## Calling Services

### initiateToasting
```bash
ros2 service call /initiateToasting std_srvs/srv/Empty
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

### setScene
```bash
ros2 service call /buildScene std_srvs/srv/Empty
```
Creates the scene with the objects in simulation so planned paths account for them.

### breadToToaster
```bash
ros2 service call /breadToToaster std_srvs/srv/Empty
```
Bread from Loaf Tray to Toaster Slot 0

### actuateLever
```bash
ros2 service call /actuateLever std_srvs/srv/Empty
```
Press the Lever on the Toaster until it clicks.

### goHome
```bash
ros2 service call /gohome std_srvs/srv/Empty
```
Sends the Franka to the Home Position

### toastToPlate
```bash
ros2 service call /toastToPlate std_srvs/srv/Empty
```
Grabs from toaster slot 0 and plates it

### openGripper
```bash
ros2 service call /openGripper std_srvs/srv/Empty
```
Opens the gripper

### closeGripper
```bash
ros2 service call /closeGripper std_srvs/srv/Empty
```
Closes the gripper

## Resources
[April Tag Generator](https://chaitanyantr.github.io/apriltag.html)

[YOLO ROS Support](https://github.com/mgonzs13/yolo_ros)

Special thanks to Zhengyang Kris Weng (https://github.com/wengmister) for providing the CAD for the April Tag holder that was affixed to the base of the robot. The files can be found here: [CAD LINK](https://github.com/wengmister/Apex-Putter/issues/2)

## CAD List
Each part should have a location for an april tag to be attached to it so we can locate it with the camera. Since we know the geometry of all the fixtures and parts we can hard-code those transformations and dynamically locate their respective tags.

### Bread End-Effector
Thick, long prongs that resemble chopsticks so we can grab the bread from the holder and the toaster with quite some Z offset. The end-effector doesn't need an april tag.\

### Loaf Holder
A holder with several slots for bread to be placed into. This is for pre-toasted bread and we can start with just 2 slots filled but we should have several slots to pick from. \
Dimensions:
 -  Slot Length: 120 mm
 -  Slot Width: 20 mm
 -  Slot Spacing (center to center): 33.3 mm
 -  Overall Length: 200 mm
 -  Overall Width: 140 mm
 -  Height: 85 mm
 -  Number of Slots: 4

April Tag Size: 50 mm

### Knife Handle
Two pieces that enclose around the actual knife handle to make it a rectangular prism. \

### Toaster Stand
An elevated stand (toaster box on its side is the perfect height) for the toaster. It will sit on the table close to the arm and off to the left (when facing the robot's forward direction) \

### Lever Platform
A platform or slab to attach to the toaster lever to make it easier to manipulate with the end-effector. Doesn't need an April tag but might be worth trying to fit a small one on there. \
Lever Dimensions:
   - Length: 35 mm
   - Width: 20.4
   - Height: 11.5 mm

April Tag Size: 70 mm

<!-- ## Toaster Dial
A dial that should press-fit or attach to the toaster dials so it's easier for the end-effector to manipulate it. \
Dial-Lever Spacing: 43 mm \
Toaster Dial Dimensions:
  - Length: 40 mm
  - Width: 10 mm
  - Height: 10 mm -->

<!-- April Tag Size:  -->

### Plate Fixture
A fixture to hold the plate and keep it on the table. This should have an april tag to locate it. \
Toast Dimensions:
  - Length: 133 mm
  - Width: 110 mm
  - Height: 20 mm

April Tag Size: 
- Toaster
    tag 0
    - total size: 20 mm - 12.5 mm
    - tag size  : 20 mm - 12.5 mm
    
- Lever
    tag 1
    - total size:: 30 mm
    - tag size  : 15 mm

- Plate
    tag 3
    - total size: 30
    - tag size  : 24
    
- Bread Holder
    tag 4
    - total size: 85 mm
    - tag size  : 85 mm

Stretch Goal
- Lever Dial
    tag 5
  - total size: 35 mm
  - tag size  : 10 mm

- Knife Handle
    tag 2
    - total size:: 30 mm
    - tag size  : 30 mm

### Toaster Dimensions
Length: 241.3 mm \
Width: 152.4 mm \
Height: 180.975 mm

## Camera (Realsense)
### Dependencies
1. Apriltags
    - install by running the following:
    `sudo apt install ros-jazzy-apriltag-ros`

2.  To launch
  `colcon clean workspace -y && colcon build && source install/setup.bash && ros2 launch realsense camera.launch.py`

## Potential Future Work
The toast_bot team's stretch goal was to be able to butter the toast after toasting, as well as actuate the dial to a preferred 'toastiness.' Additionally, further improvements could be made to allow for multiple pieces of toast to be made in one program execution.
