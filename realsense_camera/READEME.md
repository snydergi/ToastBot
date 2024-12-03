## Dependencies
1. Apriltags
    - install by running the following:
    `sudo apt install ros-jazzy-apriltag-ros`

To launch
`colcon clean workspace -y && colcon build && source install/setup.bash && ros2 launch realsense camera.launch.py`