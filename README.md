# robot-dynamics-ETH
MATLAB/Simulink Exercises for the class 'Robot Dynamics' (Prof. Marco Hutter) @ETH Zurich.

Credit for the exercises' setup, descriptions and skeleton codes goes to Prof. Hutter and his teaching assistants.


## Exercise 1: Forward, Differential and Inverse Kinematics of the ABB IRB 120
The aim of this exercise was to calculate the forward, differential and inverse kinematics of the 6-link ABB IRB 120 robot arm with fixed base seen below (with and without associated Cartesian frames). This helped practice the use of different representations of the end-effector's orientation as well as provided a tool to check whether practical implementations of the kinematics were correct. The task was to implement the functions for computing the forward and inverse kinematics using symbolic and numerical computations in MATLAB. A separate MATLAB script allowed to visualize the robot arm in 3D.

<p align="center">
  <img src=https://github.com/ischollETH/robot-dynamics-ETH/blob/main/images/ABB_IRB_120.PNG height="250" title="ABB IRB 120 Robot Arm">
  <img src=https://github.com/ischollETH/robot-dynamics-ETH/blob/main/images/ABB_IRB_120_Frames.PNG height="250" title="ABB IRB 120 Robot Arm with associated Cartesian frames">
</p>

### Exercise 1a: Forward Kinematics of the ABB IRB 120
1) Defining a vector q of generalized coordinates (arm joint angles) to describe the configuration of the ABB IRB120. The generalized coordinates should be complete (fully describe the configuration of the robot while at the same time comprising a minimal set of coordinates) and independent (each generalized coordinate must not be a function of any of the others).
2) Computing the homogeneous transformation matrices between the inertial frame (I), the base frame (0), all joint frames (1-6) as well as the end-effector frame (E).
3) Finding the function for calculating the end-effector position vector.
4) Finding the end-effector position vector for some specific vector q of generalized coordinates.
5) Finding the function for calculating the end-effector rotation matrix.
6) Finding the quaternion representing the attitude of the end-effector including functions for conversion from quaternion to rotation matrices and vice-versa, quaternion multiplication and passive rotation of a vector with a given quaternion.

### Exercise 1b: Differential Kinematics of the ABB IRB 120
1) Computing an analytical expression for the end-effector twist via the analytical expressions of the end-effector linear velocity vector and angular velocity vector as a function of the linear and angular velocities of the coordinate frames attached to each link.
2) Deriving the mapping between the generalized velocities q and the end-effector twist, namely the basic or geometric Jacobian, via derivation of the translational and rotational Jacobians of the end-effector, depending on the minimal coordinates q only.

### Exercise 1c: Inverse Kinematics of the ABB IRB 120
1) Implementing the iterative inverse kinematics algorithm (finding the joint space configuration q which corresponds to some desired pose), robust against the case for which the rotation is identity, i.e. the rotation angle is zero.
2) Using of the iterative inverse kinematics method from 1. to implement a basic end-effector pose controller for the ABB manipulator. The controller acts only on a kinematic level, i.e. it produces end-effector velocities as a function of the current and desired end-effector pose. This is essentially a kinematic motion control scheme which tracks a series of points defining a trajectory in the task-space of the robot. This includes deriving the following functional modules: a) A trajectory generator defining a discretized path that the end-effector should track and b) a kinematics-level simulator generating an updated configuration of the robot at each time-step which is then provided to the visualization for rendering.

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

## Exercise 2b: Model-based control of the ABB IRB 120
Coding of three controllers which use the dynamic model of the arm to perform motion and force tracking tasks:
1) Joint space control: a controller which compensates for the gravitational terms, and tracks a desired joint-space configuration as well as provides damping which is proportional to the measured joint velocities. The corresponding SIMULINK model as well as the arm tracking different modifications in the joint angles can be seen in the following:
<p align="center">
  <img src=https://github.com/ischollETH/robot-dynamics-ETH/blob/main/images/RD_Ex2b_JointSpaceControl.PNG width="700" title="Ex2b Joint Space Control Overview">
</p>
<p align="center">
  <img src=https://github.com/ischollETH/robot-dynamics-ETH/blob/main/images/RD_Ex2b_JointSpace.gif width="700" title="Ex2b Joint Space Controller on Robot Arm">
</p>
2) Inverse dynamics control: a controller which uses an operational-space inverse dynamics algorithm, i.e. a controller which compensates for the entire dynamics and tracks a desired motion in the operational space. Again, the derived SIMULINK model as well as a visualization of the arm first holding an end-effector orientation while changing the end-effector position, and then holding position while changing the orientation can be seen hereafter:
<p align="center">
  <img src=https://github.com/ischollETH/robot-dynamics-ETH/blob/main/images/RD_Ex2b_InverseDynamicsControl.PNG width="700" title="Ex2b Inverse Dynamics Control Overview">
</p>
<p align="center">
  <img src=https://github.com/ischollETH/robot-dynamics-ETH/blob/main/images/RD_Ex2b_InverseDynamicsControl.gif width="700" title="Ex2b Inverse Dynamics Controller on Robot Arm">
</p>
3) Hybrid force and motion control: a controller which is able to control both motion and force in orthogonal directions by the use of appropriate selection matrices. As shown in the figure below, there is a window at x = 0.1m.
<p align="center">
  <img src=https://github.com/ischollETH/robot-dynamics-ETH/blob/main/images/RD_Ex2b_HybridForceMotionControl_Illustration.PNG height="250" title="Hybrid Force an Motion Control Scenario">
</p>
The controller should wipe the window by applying a constant force on the wall in the x-axis and following a trajectory defined on the y-z plane, using the equations of motion projected to the operational space. The overview of the SIMULINK model and the resulting expected motion are shown hereafter:
<p align="center">
  <img src=https://github.com/ischollETH/robot-dynamics-ETH/blob/main/images/RD_Ex2b_HybridForceMotionControl.PNG width="700" title="Ex2b Hybrid Force Motion Control Overview">
</p>
<p align="center">
  <img src=https://github.com/ischollETH/robot-dynamics-ETH/blob/main/images/RD_Ex2b_HybridForceMotion.gif height="250" title="Ex2b Hybrid Force and Motion Controller on Robot Arm">
</p>


## Exercise 3: Controlling a Legged Robot
The objective was to implement two control algorithms for a simplified legged robot which, respectively, enable the tracking of a desired motion for the torso (i.e. base) and desired motion and force for the arm (i.e. end-effector), employing a hierarchical optimization scheme that will enable the system to execute the aforementioned tasks while also respecting a set of necessary constraints.
A planar simplification of the studied quadrupedal systems equipped with a manipulator can be seen in the following figure:
<p align="center">
  <img src=https://github.com/ischollETH/robot-dynamics-ETH/blob/main/images/RD_Ex3_Frames.PNG width="700" title="Ex3 Simplified Quadruped Robot System with Frames">
</p>
The exercise restricts motion to the X-Z plane, each pair of front and hind legs is modelled by a single articulated limb, which supports a free-floating base. Additionally, an articulated arm limb is attached for manipulation tasks. The underlying system is shown in the following schematic:

<p align="center">
  <img src=https://github.com/ischollETH/robot-dynamics-ETH/blob/main/images/RD_Ex3_SystemOverview.PNG width="700" title="Ex3 System Overview Schematic">
</p>
The method used consists in re-formulating the Hierarchical Optimization (HO) problem as a Quadratic Program (QP) to compute optimal torques, given a set of carefully selected costs and constraints.

1) Floating-Base Motion Control: Implement a controller to track Cartesian pose and velocity references for the base of the robot. These references should result in the base moving in a circular trajectory in the X-Z plane, while maintaining a constant attitude parallel with the floor. This resulting movement can indeed be observed in the following visualization:
<p align="center">
  <img src=https://github.com/ischollETH/robot-dynamics-ETH/blob/main/images/RD_Ex3_Floating_Base_Control.gif width="700" title="Ex3 Floating Base Controller on Quadruped">
</p>
2) Hybrid Force-Motion Control: Extend and adapt the formulation of the previous problem, but now also control the interaction force acting between the end-effector and the wall. Essentially, the end-effector is used to additionally push against a wall at a fixed spot. The interaction forces of the robot gripper with the wall are clearly observable in the following capture:

<p align="center">
  <img src=https://github.com/ischollETH/robot-dynamics-ETH/blob/main/images/RD_Ex3_Hybrid_Force_Motion_Control.gif width="700" title="Ex3 Hybrid Force Motion Controller on Quadruped">
</p>


## Exercise 4: Modeling and Control of a Multicopter
The aim of this exercise was to derive the full dynamic model of a hexacopter and to implement control algorithms to stabilize the platform attitude and to achieve velocity reference tracking. The hexacopter model with all associated frames, spinning directions as well as forces and torques acting on the vehicle and inertial and body frames can be seen here:
<p align="center">
  <img src=https://github.com/ischollETH/robot-dynamics-ETH/blob/main/images/RD_Ex4_Hexacopter_Frames.PNG width="700" title="Ex4 Hexacopter Model">
</p>

1) The full dynamic model of a hexacopter was derived on paper assuming that the vehicle is a rigid body. The dynamic model had to be represented as a set of ordinary differential equations.
2) An attitude PD controller and velocity P controller were designed and evaluated using MATLAB and SIMULINK. The control structure is depicted hereafter, where the vehicle model is divided into 2 subsystems, namely the attitude dynamics and the translational dynamics.
<p align="center">
  <img src=https://github.com/ischollETH/robot-dynamics-ETH/blob/main/images/RD_Ex4_Control_Structure.PNG width="700" title="Ex4 Hexacopter Control Structure">
</p>
The outer loop controller (velocity controller) generates commands for the inner loop controller (attitude controller). The commands (total thrust and moments) are converted into rotor speeds. Resulting vehicle dynamics plots given some desired velocities in the body frame simulated in SIMULINK can be seen in the following:

<p align="center">
  <img src=https://github.com/ischollETH/robot-dynamics-ETH/blob/main/images/RD_Ex4_Vel.PNG width="700" title="Ex4 Vehicle Dynamics Simulation">
</p>


## Exercise 5: Fixed-wing UAV Simulation and Control
The aim of this exercise was the familiarization with typical fixed-wing UAV dynamics, how the model derived in theory may be simulated, and how one may control the various low- and high-level states of the vehicle. The overall control architecture can be seen in this figure:
<p align="center">
  <img src=https://github.com/ischollETH/robot-dynamics-ETH/blob/main/images/RD_Ex5_Control_Architecture.PNG width="700" title="Ex5 Fixed-Wing UAV Control Architecture">
</p>

1) Find a set of manual control deflections (and throttle setting) in order to stabilize straight, level, and steady flight in open loop for the body-x-axis airspeed component.
2) Implement a cascaded-PID control scheme to track attitude setpoints and damp attitude rates, airspeed and altitude control via a total energy control system (TECS) as well as lateral-directional position control (L1 position control).
With all these implemented, the UAV is able to accurately follow a circular motion, even under the influence of external disturbances such as wind. The resulting motion can be observed in the graph on the bottom right, along with other tracked variables:
<p align="center">
  <img src=https://github.com/ischollETH/robot-dynamics-ETH/blob/main/images/RD_Ex5_Plots.PNG width="700" title="Ex5 UAV Circle Following Plots">
</p>
