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