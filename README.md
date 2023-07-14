# robot-dynamics-ETH
MATLAB/Simulink Exercises for the class 'Robot Dynamics' (Prof. Marco Hutter) @ETH Zurich.

Credit for the exercises' setup, descriptions and skeleton codes goes to Prof. Hutter and his teaching assistants.

## Exercise 1: Forward, Differential and Inverse Kinematics of the ABB IRB 120
The aim of this exercise was to calculate the forward, differential and inverse kinematics of the 6-link ABB IRB 120 robot arm with fixed base. This helped practice the use of different representations of the end-fector's orientation as well as provided a tool to check whether practical implementations of the kinematics were correct. The task was to implement the functions for computing the forward and inverse kinematics using symbolic and numerical computations in MATLAB. A separate MATLAB script allowed to visualize the robot arm in 3D.


### Exercise 1a: Forward Kinematics of the ABB IRB 120
1) Deefining a vector q of generalized coordinates (arm joint angles) to describe the configuration of the ABB IRB120. The generalized coordinates should be complete (fully describe the configuration of the robot while at the same time comprising a minimal set of coordinates) and independent (each generalized coordinate must not be a function of any of the others).
2) Computing the homogeneous transformation matrices between the inertial frame (I), the base frame (0), all joint frames (1-6) as well as the end-effector frame (E).
3) Finding the function for calculating the end-effector position vector.
4) Finding the end-effector position vector for some specific vector q of generalized coordinates.
5) Finding the function for calculating the end-effector rotation matrix.
6) Finding the quaternion representing the attitude of the end-effector including functions for convertsion from quaternion to rotation matrices and vice-
versa, quaternion multiplication and passive rotation of a vector with a given quaternion.

### Exercise 1b: Differential Kinematics of the ABB IRB 120
1) Computing an analytical expression for the end-effector twist via the analytical expressions of the end-effector linear velocity vector and angular velocity vector as a function of the linear and angular velocities of the coordinate frames attached to each link.
2) Deriving the mapping between the generalized velocities q and the end-effector twist, namely the basic or geometric Jacobian, via derivation of the translational and rotational Jacobians of the end-effector, depending on the minimal coordinates q only.

### Exercise 1c: Inverse Kinematics of the ABB IRB 120
1) Implementing the iterative inverse kinematics algorithm (finding the joint space configuration q which corresponds to some desired pose), robust against the case for which the rotation is identity, i.e. the rotation angle is zero.
2) Using of the iterative inverse kinematics method from 1) to implement a basic end-effector pose controller for the ABB manipulator. The controller acts only on a kinematic level, i.e. it produces end-effector velocities as a function of the current and desired end-effector pose. This is essentially a kinematic motion control scheme which tracks a series of points defining a trajectory in the task-space of the robot. This includes deriving the following functional modules: a) A trajectory generator defining a discretized path that the end-effector should track and b) a kinematics-level simulator generating an updated configuration of the robot at each time-step which is then provided to the visualization for rendering.

### Exercise 1: Result
As a result of implementating all the kinematics correctly, an animation of the robot accurately following a desired path and linear velocity can be seen hereafter:
<p align="center">
  <img src=https://github.com/ischollETH/robot-dynamics-ETH/blob/main/images/RD_Ex1c.gif width="350" title="Ex1 Result Path Following Robot Arm">
</p>

## Exercise 2: 

