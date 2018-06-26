This is the implementation of an Extended Kalman Filter that I designed for an assignment in ME233 at UC Berkeley.  It uses a simple dynamic bicycle model based on velocity, heading angle, and wheelbase length, as well as a position measurment model based on prior estimated position, wheelbase length, and estimated heading angle.

Given Constants:
Radius of wheel: 0.425m (+/- 5%)
Wheelbase: 0.8m (+/- 10%)
gear ratio (rear wheel ang velocity:pedal speed): 5

Given Variables at a given time step (if no variable present, it is passed as NaN):
1. Current time (s)
2. Current steering angle (radians)
3. Current pedal speed (rad/s)
4. Current x position measurment
5. Current y position measurment
6. True X postion in meters (all NaN except final value)
7. True Y postion in meters(all NaN except final value)
8. True heading angle in radians (all NaN except final value)


File Descriptions
estInitialize.m: initializes estimator internal state that is passed to the EKF at each time step
estRun.m: This is the EKF.  It runs a prior and measurement update at each time step
main.m: This runs a single test case on the EKF and provides performance data based on how close the estimate is on the final true values (all other true values hidden)
runAllTests.m: Same as main.m, but it instead runs all tests and reports the mean error and variance across all tests
evaluateMultiple.m: This runs the evaluation tests on the EKF and provides a performance score based on the error between the true state and estimated state throughout the entire dataset (as opposed to just the final value, like in the tests)
