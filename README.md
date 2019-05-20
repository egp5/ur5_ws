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
Converts joint positions into end effector position

$ `cd ur5_ws-master/src/forward-kinematics/scripts`

$ `chmod +x fwk.py`

$ `roslaunch forward_kinematics fwk.launch`

![fwk](https://user-images.githubusercontent.com/37616724/57997864-885a4a80-7aec-11e9-8f12-cf11915331ae.gif)


Toggle the RobotModel in RVIZ to understand the Coordinate frames

## Start the Cartesian Control simulation 
Converts cartesian position and orientation of the interactive markers into joint velocities of the UR5 robot to make the end effector align with the interactive marker

$ `roslaunch cartesian_control cartesianControl.launch`

Under 'Global' options of Display panel in Rviz change 'Fixed Frame' to 'World'

![fixed_frame_cropped](https://user-images.githubusercontent.com/37616724/58003646-d415ef80-7afe-11e9-9c89-20be29ddda78.png)


In the Dispaly panel select 'Add' --> 'InteractiveMarkers' --> 'Ok'

Under 'InteractiveMarkers' of Display panel in Rviz change 'Update Topic' to '/control_markers/update'

![update_topic](https://user-images.githubusercontent.com/37616724/58003754-10495000-7aff-11e9-9fc8-a0a50ae80b55.png)


Change the position and orientation of interactive markers so that the end effector of UR5 follows.


![cartesian_control](https://user-images.githubusercontent.com/37616724/58004223-5c48c480-7b00-11e9-9853-3241891befa6.gif)


