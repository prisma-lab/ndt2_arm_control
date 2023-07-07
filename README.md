# ndt2_arm_control
This package contains a ROS controller for a 6 DoF manipulator to simulate with Gazebo. A parallel vision/force control technique is implemented.

The controller is divided in three steps:

### Approaching phase
The arm is considered in a rest configuration regulating its pose to preserve the initial joints position
### IBVS
When the drone is regulating its position in the image space, the visual servoing for the arm is triggered: 4 virtual features are considered on the End-Effector and the control objective is to nullify the error in the image space
### Parallel IBVS/Force control
When a contact with the environment arise or when the End-Effector is near to it, the parallel controller is triggered. The selection matrix P imposes the axis to be controlled with the IBVS algorithm and the ones to be controlled with the direct force controller. When the pushing phase is completed and the force is regulated to a desired value the aerial manipulator starts to slide on the surface without losing the contacts  

