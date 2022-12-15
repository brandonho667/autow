# Autow

Automatic Towing Project for ECE148 - Autonomous Vehicles

[![tow.jpg](https://upload.wikimedia.org/wikipedia/commons/0/06/Autow.jpg)](https://www.youtube.com/watch?v=caYrD2hRw2U)
<p align=center> Autow Demo Video and [presentation](https://docs.google.com/presentation/d/1bZEgKXNtt-FZTL9H_7P-j5dTLegfVZknjA--xCtpJQ8/edit?usp=sharing) </p>

## Features

- Joystick RC car control
- AR Tag Detection and Pose Estimation
- Autonomous tow hitch lineup to trailer with dynamic steering
- Asynchronous driver cancellation during autonomous hitching

## Steps for running

1. `cd` to your `ros2_ws`
2. `colcon build --packages-select autow_ros`
3. `source install/setup.bash`
4. `ros2 launch autow_ros autow_launch.yaml`

## Helpful Docker commands on Robocar

- `robocar_docker ros_container`
- `` docker start  `docker ps -q -l`  ``
- `` docker attach `docker ps -q -l`  ``

## Audio on Jetson Nano

- `sudo pulseaudio --system -D` to start PulseAudio
- `pax11publish -r` to enable connection to PulseAudio
