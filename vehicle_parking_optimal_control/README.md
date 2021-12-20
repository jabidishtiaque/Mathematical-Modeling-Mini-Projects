# Applying Optimal Control Methods for Reverse Parking of a Vehicle

Maneuvering assistance is one of the most focused research areas in recent years. Many Advanced Driver Assistance Systems (ADAS) have been introduced during the last decades. This project is based on the kinematics of backward parking of a car and trailer. For achieving high precision in driving and brisk motions of the vehicle, a simple mathematical modeling and simulation of the real problem is done using MATLAB. A kinematic model is sufficient to describe the behavior of the
vehicle when there is low speed maneuver on dry road surface that means there is no slip condition. The whole work is divided in three parts.

  1. Kinematic modeling of vehicle and vehicle with trailer (equations)
     - Deriving the dynamic equations for a vehicle
     - Deriving the dynamic equations for a vehicle and trailer system
  2. Finding optimal control equations to make the vehicle follow the trajectory
     - Modeling the dynamic equations on MATLAB
     - Applying optimal control to the model to make the vehicle follow a path
  3. Designing the environment as well as adding car and trailer geometry
     - Applying geometry around the vehicle
     - Designing parking lot environment and to make sure the vehicle does not hit any obstacles

### Kinematic Bicycle Models

For capturing the vehicle motion assuming no slip condition because of slow maneuver, we have adopted kinematic bicycle model. The vehicle is assumed to operate on a flat surface and front wheel steered with perfect **Ackerman steering**. 

<img src="https://github.com/jabidishtiaque/Mathematical-Modeling-Mini-Projects/blob/main/vehicle_parking_optimal_control/images/vehicle.png" width="35%"></img> <img src="https://github.com/jabidishtiaque/Mathematical-Modeling-Mini-Projects/blob/main/vehicle_parking_optimal_control/images/vt.png" width="45%"></img> 

### Optimal Control

Optimal control theory is a branch of applied mathematics that deals with finding a control law for a dynamical system over a period of time such that an objective function is optimized. The general structure of an optimal control problem is straightforward. In the simplest version, there is a given dynamic
system (linear or nonlinear, discrete-time or continuous-time) for which input functions can be specified. There is also an objective function whose value is determined by system behavior, and is in some sense a measure of the quality of that behavior. 

I have formulated the problem as a continuous-time system. The following steps were taken during formulation of the problem.
  - Form the Hamiltonian for the problem.
  - Write the adjoint differential equation, transversality boundary condition, and the optimality condition. Now there are three unknowns, u<sup>s</sup>, x<sup>s</sup> and z.
  - Try to eliminate u<sup>s</sup> by using the optimality equation H<sub>u</sub> = 0, i.e., solve for u<sup>s</sup> in terms of x<sup>s</sup> and z.
  - Solve the two differential equations for x<sup>s</sup> and z, with two boundary conditions, substituting u<sup>s</sup> in the differential equations with the expression for the optimal control from the previous step.
  - After finding the optimal state and adjoint, solve for the optimal control.

#### Objective function

<img src="https://github.com/jabidishtiaque/Mathematical-Modeling-Mini-Projects/blob/main/vehicle_parking_optimal_control/images/objfunc.png" width="60%"></img> 

#### Hamiltonian

<img src="https://github.com/jabidishtiaque/Mathematical-Modeling-Mini-Projects/blob/main/vehicle_parking_optimal_control/images/hamilton.png" width="60%"></img>

#### Adjoint or Costate equations

<img src="https://github.com/jabidishtiaque/Mathematical-Modeling-Mini-Projects/blob/main/vehicle_parking_optimal_control/images/adjoints.png" width="60%"></img>



### Numerical Methods

#### Constraints

<img src="https://github.com/jabidishtiaque/Mathematical-Modeling-Mini-Projects/blob/main/vehicle_parking_optimal_control/images/constraints.png" width="60%"></img>

#### Vehicle Geometry

<img src="https://github.com/jabidishtiaque/Mathematical-Modeling-Mini-Projects/blob/main/vehicle_parking_optimal_control/images/geometry.png" width="60%"></img>


### Priliminary Results

#### Parking Around a Large Turning Radius

<img src="https://github.com/jabidishtiaque/Mathematical-Modeling-Mini-Projects/blob/main/vehicle_parking_optimal_control/images/large1.jpg" width="33%"></img> <img src="https://github.com/jabidishtiaque/Mathematical-Modeling-Mini-Projects/blob/main/vehicle_parking_optimal_control/images/large3.jpg" width="33%"></img> <img src="https://github.com/jabidishtiaque/Mathematical-Modeling-Mini-Projects/blob/main/vehicle_parking_optimal_control/images/large7.jpg" width="66%"></img>

#### Parking Around a Sharp Corner

<img src="https://github.com/jabidishtiaque/Mathematical-Modeling-Mini-Projects/blob/main/vehicle_parking_optimal_control/images/sharp1.jpg" width="33%"></img> <img src="https://github.com/jabidishtiaque/Mathematical-Modeling-Mini-Projects/blob/main/vehicle_parking_optimal_control/images/sharp3.jpg" width="33%"></img> <img src="https://github.com/jabidishtiaque/Mathematical-Modeling-Mini-Projects/blob/main/vehicle_parking_optimal_control/images/sharp7.jpg" width="66%"></img>


#### Reverse Parallel Parking

<img src="https://github.com/jabidishtiaque/Mathematical-Modeling-Mini-Projects/blob/main/vehicle_parking_optimal_control/images/par1.jpg" width="33%"></img> <img src="https://github.com/jabidishtiaque/Mathematical-Modeling-Mini-Projects/blob/main/vehicle_parking_optimal_control/images/par3.jpg" width="33%"></img> <img src="https://github.com/jabidishtiaque/Mathematical-Modeling-Mini-Projects/blob/main/vehicle_parking_optimal_control/images/par7.jpg" width="66%"></img>



*After the initial simulations of the vehicle, I have tried similar approach with vehicle and trailer system. But I need more constraints to make the system work perfectly. I did not complete the whole system model while I was at University. Later on I just got busy with other projects. A shortcoming of this work is the environment model. Implementation of the environmental constraints to the model is yet to be done. Apart from these, the model works very smoothly with any kind of path for the vehicle. This research could be expanded further by adding more variables in the model such as wheel friction and vehicle and trailer weights.*




