# Bicycle EKF
This is an implementation of an Extended Kalman Filter that I designed for an assignment in ME233 at UC Berkeley.  It uses a simple dynamic bicycle model based on velocity, heading angle, and wheelbase length, as well as a position measurment model based on prior estimated position, wheelbase length, and estimated heading angle.

## Given Constants
Radius of wheel: 0.425m (+/- 5%)
Wheelbase: 0.8m (+/- 10%)
gear ratio (rear wheel ang velocity:pedal speed): 5

## Given Variables at each time step (if no variable present, it is passed as NaN):
1. Current time (s)
2. Current steering angle (radians)
3. Current pedal speed (rad/s)
4. Current x position measurment
5. Current y position measurment
6. True X postion in meters (all NaN except final value)
7. True Y postion in meters(all NaN except final value)
8. True heading angle in radians (all NaN except final value)

## Designing and Testing the EKF
Tune the EKF in estInitialize.m (initial state) and estRun.m (process model and noise, measurement model and noise)
Run estRun.m to test the EKF on a single test case (in testData) and get performance stats and graphs
Run runAllTests.m to test the EKF on all test cases (in testData) and get average performance stats
Run evaluateMultiple.m to evaluate the performance of the EKF with the evaluationData tests and receive comprehensive performance stats
