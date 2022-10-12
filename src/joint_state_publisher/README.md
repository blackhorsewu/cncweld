# joint_state_publisher
=====================

catkinized version of David Lu!!'s package

This package publishes sensor_msgs/JointState messages for a robot. The package reads the robot_description parameter, finds all of the non-fixed joints and publishes a JointState message with all those joints defined.

There are four possible sources for the value of each JointState.  
1. Values directly input through the GUI, which I have taken out.
2. JointState messages that the node subscribes to (see the source_list parameter)
3. The value of another joint. Currently this is implemented through the dependent_joints parameter. In the future it will use the mimic tag.
4. The default value. The default value is zero, or if zero is not permissible value, (max+min)/2.

If a GUI is present, the package displays the joint positions in a window as sliders. Each slider is set to the joints' min and max limits, except for continuous joints, which range from -Pi to +Pi.

This node also lets you constrain certain joints to be equal to other joints, or multiples of other joints. For example, if you had two separate joints for two wheels, but wanted them to turn at the same rate, you could constrain the rotation of one to be equal to the other. Or, if you had two unequally sized pulleys, and wanted to have one pulley drive the other, you could constrain the second pulley to turn 3 times as fast as the other. As mentioned above, this is set with a parameter currently, but in the future will use the mimic tag information.

## joint_controller

Publishes joint states, read from GUI. Can be used in conjunction with the robot_state_publisher node to publish joint states and transforms.