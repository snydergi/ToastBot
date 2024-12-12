# ToastBot
## Thermal Optimization and Actuation of Sliced Toast Robot (T.O.A.S.T Bot)
Authors: Sharwin Patil, Grayson Snyder, Asa Rogers, and Tony Shilati

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

# CAD List
Each part should have a location for an april tag to be attached to it so we can locate it with the camera. Since we know the geometry of all the fixtures and parts we can hard-code those transformations and dynamically locate their respective tags.
## Bread End-Effector
Thick, long prongs that resemble chopsticks so we can grab the bread from the holder and the toaster with quite some Z offset. The end-effector doesn't need an april tag.\
Dimensions:

## Loaf Holder
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

## Knife Handle
Two pieces that enclose around the actual knife handle to make it a rectangular prism. \
Printed Handle Dimensions:
 - Length:
 - Width:
 - Height: 

## Toaster Stand
An elevated stand (toaster box on its side is the perfect height) for the toaster. It will sit on the table close to the arm and off to the left (when facing the robot's forward direction) \
Dimensions:
  - Length:
  - Width:
  - Height:

April Tag Size: 

## Lever Platform
A platform or slab to attach to the toaster lever to make it easier to manipulate with the end-effector. Doesn't need an April tag but might be worth trying to fit a small one on there. \
Lever Dimensions:
   - Length: 35 mm
   - Width: 20.4
   - Height: 11.5 mm

April Tag Size: 70 mm

## Toaster Dial
A dial that should press-fit or attach to the toaster dials so it's easier for the end-effector to manipulate it. \
Dial-Lever Spacing: 43 mm \
Toaster Dial Dimensions:
  - Length: 40 mm
  - Width: 10 mm
  - Height: 10 mm
Printed Dial Dimensions:
 - Diameter:
 - Length:

April Tag Size: 

## Plate Fixture
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
    
- Knife Handle
    tag 2
    - total size:: 30 mm
    - tag size  : 30 mm

- Plate
    tag 3
    - total size: 30
    - tag size  : 24
    
- Bread Holder
    tag 4
    - total size: 85 mm
    - tag size  : 85 mm

Stretch
- Lever Dial
    tag 5
  - total size: 35 mm
  - tag size  : 10 mm

## Toaster Dimensions
Length: 241.3 mm \
Width: 152.4 mm \
Height: 180.975 mm

# Camera (Realsense)
## Dependencies
1. Apriltags
    - install by running the following:
    `sudo apt install ros-jazzy-apriltag-ros`

To launch
`colcon clean workspace -y && colcon build && source install/setup.bash && ros2 launch realsense camera.launch.py`
