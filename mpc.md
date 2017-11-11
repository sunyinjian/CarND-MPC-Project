# MPC Controller
# Vehicle Model
The kinematic vehicle model is used in the simulation. The model is defined as:

![](./vehicle_model.png)

# N & dt
The prediction length N is set to 10 steps to save computation time and predict enough trajectory. The elapsed duration is set to 0.1s. The elapsed duration is set according to the time delay. 

# Polynomial Fitting and Preprocessing
The waypoints are fitted by a 3rd polynomial. Transform the velocity unit from mph into m/s. The range of throttle is -1.0~+1.0. The throttle is transformed into acceleration by multiplying a ratio of 10.0. That is, if the contrrol value of throttle is 1.0, the acceleration of the vehicle is 10.0m/ss. The range of steering value is -1.0~+1.0. The steering control value is transformed into the front wheels angle by multiplying a ratio of 0.436332. That is, if the control value of steer is 1.0, the steering angle of front wheel is 25.0 degrees (value in radians); Apply the negative sign to the steering angle of front wheel.

# Latency
Assume that the steer and throttle are not changed during the 0.1s latency. Predict the state forward at the latency time before feeding into the solver.

# Reference velocity
The reference velocity is set to 70.0mph.

