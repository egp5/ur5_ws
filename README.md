# ur5_ws
### This repo contains ROS-Kinetic packages for kinematic simulations of Universal Robot UR5

BUILD AND SOURCE THE SETUP FILES (NEED TO BE DONE EVERYTIME YOU OPEN A TERMINAL)

$ `cd ur5_ws-master` 

$ `catkin_make` 

$ `source devel/setup.bash` 

## Start the simulation with GUI to control joint values to get a feel of the simulation

$ `roslaunch ur_description demo.launch`

Play around with the sliders and toggle the RobotModel in Rviz to understand the co-ordinate frames

![demo_FK_cropped](https://user-images.githubusercontent.com/37616724/57997667-8c399d00-7aeb-11e9-84f3-3692596d6cf3.png)

## Start the forward kinematics simulation 
Convert joint positions into end effector position

$ `cd ur5_ws-master/src/forward-kinematics/scripts`

$ `chmod +x fwk.py`

$ `roslaunch forward_kinematics fwk.launch`

![fwk](https://user-images.githubusercontent.com/37616724/57997864-885a4a80-7aec-11e9-8f12-cf11915331ae.gif)


Toggle the RobotModel in RVIZ to understand the Coordinate frames
