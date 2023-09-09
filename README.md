# Drone Dash 2023
This is the repository for Drone Dash @ Summer of Innovation 2023, IIT Dharwad.

## Objective
The objective of Drone Dash is to design a obstacle avoidance algorithm for a drone. There is an onboard color and depth camera which can be used for this task. You may use any algorithm you like. The goal is to reach the end of the track in the shortest time possible.

## Development Environment
You will be developing the algorithm for use with the [PX4 Autopilot Stack](https://px4.io/) in [ROS](https://www.ros.org/). The PX4 Autopilot is a flight control software stack for drones. ROS is a robotics middleware that provides a set of tools and libraries for developing robot applications.

You will be using the [Gazebo](https://classic.gazebosim.org/) simulator to test your algorithm. Gazebo is a 3D robotics simulator that provides a physics engine, high-quality graphics, and convenient programmatic and graphical interfaces.

Instructions for setting up the development environment are provided in the [Installation Guide](./docs/Installation.md).

## Demo Script
We have provided a [demo script](./scripts/demo_node.py) to get you started. The script gets the drone to takeoff and then move forward and backward in a loop. You can launch a simulation with the demo script using the following command.
```bash
roslaunch dronedash demo.launch
```

To see the RGB and depth images from the camera in RVIZ, set the `rviz` parameter to `true`.
```bash
roslaunch dronedash demo.launch rviz:=true
```
## image_subscriber.py
This node takes images from the onboard RGBD camera and converts them into arrays. Then it calculates distance from depth images. The drone is able to avoid the obstacles and navigate through the worlds. A further development is adding landing algorithm. I'm currently working on adding SLAM to this project.
