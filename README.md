# Leonardo Drone Contest 2022

This is the software for the third challenge "Leonardo Drone Contest" of 2022. 

The package includes several ROS nodes for the various tasks that the drone needs to 
perform during the challenge.

## Prerequisites

The simulation and the autopilot are based on PX4 v1.12.3 and ROS Melodic/Noetic.

Clone the repository with the simulator and the PX4 firmware 

```bash
git clone --recursive https://github.com/salvamarce/PX4_drone_contest.git PX4-Autopilot
```

and refer to the official [installation guide of PX4](https://docs.px4.io/master/en/dev_setup/building_px4.html) for the Ubuntu developer toolchain.

## Usage

To start the action server, type

```bash
roslaunch drone_contest_2022 action_server.launch
```
This will start:
  
  - The "setPointPublisher" that publishes the setpoints to "/mavros/setpoint_raw/local" in order to use the offboard flight mode (the position feedback is also needed, as default the program reads that from "/mavros/vision_pose/pose" )
  - The action server 

To use the actions you can create your own action clients or publish a goal from a new terminal, for example


```bash
rostopic pub /takeOff/goal drone_contest_2022/takeOffActionGoal 
"header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  altitude_setpoint: 2.0
  duration: 2.0" 
 
```

## Contributing
Pull requests are welcome. For major changes, please open an issue first to discuss what you would like to change.
