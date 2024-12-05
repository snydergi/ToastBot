# ToastBot
## Thermal Optimization and Actuation of Sliced Toast Robot (T.O.A.S.T Bot)

## Getting Setup
This repository depends on the [moveit API](https://github.com/ME495-EmbeddedSystems/moveitapi-group5) developed by the same authors. In order to maintain this project and the API separately, it is submoduled in this repo, and must be updated in order to include.

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

# Resources
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
 -  Slot Depth: 80 mm
 -  Slot Spacing: 24 mm
 -  Number of Slots:

April Tag Size:

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

April Tag Size: 

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
    - total size:: 85 mm
    - tag size  : 85 mm

Stretch
- Lever Dial
    tag 5
  - total size:: 35 mm
  - tag size  : 10 mm