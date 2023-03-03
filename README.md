# Wheeled Invert Pendulum Project , ROS2-Gazebo

## wip model

- This model has two wheels links and a haed link.
- Imu sensor integrated into head link.
- Wheels can be controlled by effort controller of ROS2 control.
- PID controller is used to keep wip balance with IMU sensor data.

![wip model in gazebo](doc/pics/wip_model_in_gazebo.png)

## gazebo simulation

### without balance controller

wip can't stand without balance controller.
wip fall down after about 2 seconds started simulation.

https://user-images.githubusercontent.com/46311832/222610681-1fdaaf89-af6f-4bae-8df3-d2397a96b070.mp4

### with balance controller

https://user-images.githubusercontent.com/46311832/222610702-3989f378-2768-4cdd-9e44-80286999b915.mp4

wip can stand with balance controller.

## launch file

pkg : bringup
file : start_process.launch.py
