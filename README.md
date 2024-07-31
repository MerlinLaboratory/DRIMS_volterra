# DRIMS Volterra

This repo contains the software to partecipate to the 2024 DRIMS Summer School.

## Installation

1. Follow the instruction at https://github.com/MerlinLaboratory/ABB_omnicore_ros_driver.git to install the drivers of the robots. (**Important:** use the branch **DRIMS-Volterra-2024**)
2. Clone this repository inside your catkin_ws
3. ```rosdep install --from-paths src --ignore-src -r -y```
4. ```catkin build -cs```

## Gazebo Simulation
If there are no errors you are ready to proceed to launch the Gazebo simulation of the robot. You can select between the Yumi and GoFa arm by changing the launch param (robot:=yumi for the YuMi arm, or robot:=gofa for GoFa arm:
```bash
  source devel/setup.bash
  roslaunch drims_dice_demo arm_gazebo.launch robot:=yumi
```
### Motion Planning
Launch the PosePlan, SlerpPlan and JointPlan ROS Services server:
```bash
  source devel/setup.bash
  roslaunch abb_wrapper_control launchControlServer.launch robot:=yumi
```




