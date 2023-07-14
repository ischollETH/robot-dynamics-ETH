# robot-dynamics-ETH
MATLAB/Simulink Exercises for the class 'Robot Dynamics' (Prof. Marco Hutter) @ETH Zurich.

Credit for the exercises' setup, descriptions and skeleton codes goes to Prof. Hutter and his teaching assistants.

## Exercise 1: Forward, Differential and Inverse Kinematics of the ABB IRB 120
The aim of this exercise was to calculate the forward, differential and inverse kinematics of the 6-link ABB IRB 120 robot arm with fixed base seen below (with and without associated Cartesian frames). This helped practice the use of different representations of the end-fector's orientation as well as provided a tool to check whether practical implementations of the kinematics were correct. The task was to implement the functions for computing the forward and inverse kinematics using symbolic and numerical computations in MATLAB. A separate MATLAB script allowed to visualize the robot arm in 3D.

<p align="center">
  <img src=https://github.com/ischollETH/robot-dynamics-ETH/blob/main/images/ABB_IRB_120.PNG width="350" title="ABB IRB 120 Robot Arm">
  <img src=https://github.com/ischollETH/robot-dynamics-ETH/blob/main/images/ABB_IRB_120_Frames.PNG width="350" title="ABB IRB 120 Robot Arm with associated Cartesian frames">
</p>

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
  <img src=https://github.com/ischollETH/robot-dynamics-ETH/blob/main/images/RD_Ex1c.gif width="700" title="Ex1 Result Path Following Robot Arm">
</p>

## Exercise 2: Dynamics and Model-based control of the ABB IRB 120
The aim of this exercise was to develop a tool which implements the equations of motion for the same ABB robot arm, as well as implementing control algorithms focused on model-based control schemes.

## Exercise 2a: Dynamics of the ABB IRB 120
1) Generating all the quantities (mass matrix, Coriolis, centrifugal and gravity terms) which are used in the equations of motion, as well as
the total mechanical energy of the system. 
2) This can be validated via the calculation and tracking of the total energy (= Hamiltonian, e.g., if no external forces are acting on the system, the total mechanical energy should remain constant over time).

The following visualizes the robot moving when certain torques are applied to different joints, with the corresponding effect on the total energy:<p align="center">
  <img src=https://github.com/ischollETH/robot-dynamics-ETH/blob/main/images/RD_Ex2a.gif width="700" title="Ex2a Result Robot Arm Torques and Total Energy">
</p>

## Exercise 2a: Dynamics of the ABB IRB 120
1) Generating all the quantities (mass matrix, Coriolis, centrifugal and gravity terms) which are used in the equations of motion, as well as
the total mechanical energy of the system. 
2) This can be validated via the calculation and tracking of the total energy (= Hamiltonian, e.g., if no external forces are acting on the system, the total mechanical energy should remain constant over time).

The following visualizes the robot moving when certain torques are applied to different joints, with the corresponding effect on the total energy:
<p align="center">
  <img src=https://github.com/ischollETH/robot-dynamics-ETH/blob/main/images/RD_Ex2a.gif width="700" title="Ex2a Result Robot Arm Torques and Total Energy">
</p>

## Exercise 2b: Model-based control of the ABB IRB 120
Coding of three controllers which use the dynamic model of the arm to perform motion and force tracking tasks:
1) Joint space control: a controller which compensates for the gravitational terms, and tracks a desired joint-space configuration as well as provides damping which is proportional to the measured joint velocities. Visualized in the following:
<p align="center">
  <img src=https://github.com/ischollETH/robot-dynamics-ETH/blob/main/images/RD_Ex2b_JointSpace.gif width="350" title="Ex2b Joint Space Controller on Robot Arm">
</p>
2) Inverse dynamics control: a controller which uses an operational-space inverse dynamics algorithm, i.e. a controller which compensates for the entire dynamics and tracks a desired motion in the operational space. Visualized in the following:
<p align="center">
  <img src=https://github.com/ischollETH/robot-dynamics-ETH/blob/main/images/RD_Ex2b_InverseDynamicsControl.gif width="350" title="Ex2b Inverse Dynamics Controller on Robot Arm">
</p>
3) Hybrid force and motion control: a controller which is able to control both motion and force in orthogonal directions by the use of appropriate selection matrices. As shown in the figure below, there is a window at x = 0.1m. The controller should wipe the window. by applying a constant force on the wall in the x-axis and following a trajectory defined on the y-z plane, using the equations of motion projected to the operational space. Scenario and result visualized in the following:
<p align="center">
  <img src=https://github.com/ischollETH/robot-dynamics-ETH/blob/main/images/RD_Ex2b_HybridForceMotionControl_Illustration.PNG width="350" title="Hybrid Force an Motion Control Scenario">
  <img src=https://github.com/ischollETH/robot-dynamics-ETH/blob/main/images/RD_Ex2b_HybridForceMotionControl.gif width="350" title="Ex2b Hybrid Force and Motion Controller on Robot Arm">
</p>

