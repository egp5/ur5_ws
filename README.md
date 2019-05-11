# ur5_ws
### This repo contains ROS-Kinetic packages for kinematic simulations of Universal Robot UR5

BUILD AND SOURCE THE SETUP FILES (NEED TO BE DONE EVERYTIME YOU OPEN A TERMINAL)

$ `cd ur5_ws-master` 

$ `catkin_make` 

$ `source devel/setup.bash` 

## Start the simulation with GUI to control joint values to get a feel of the simulation

$ `roslaunch ur_description demo.launch`

Play around with the sliders and toggle the RobotModel in Rviz to understand the co-ordinate frames

## Start the forward kinematics simulation 
Convert joint positions into end effector position
$ `cd ur5_ws-master/src/forward-kinematics/scripts`
$ `chmod +x fwk.py`
$ `roslaunch forward_kinematics fwk.launch`

Toggle the RobotModel in RVIZ to understand the Coordinate frames
