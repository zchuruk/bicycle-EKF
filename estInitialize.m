function internalState = estInitialize
% This function generates the internal state of the estimator at time 0. 

% Interal state is like a structure, with the first three elements the
% positions x, y; the angle theta; and our favourite colour.

% Initial State Variances
varX0 = 0.4^2;
varY0 = 0.4^2;
varTheta0 = (0.35/2)^2;
varR0 = (.05/2*.425)^2;
varB0 = (.1/2*.8)^2;
varN0 = 0;
varSteer = 0;
varPS = 0;

% Initialize States
internalState.x = 0;
internalState.y = 0;
internalState.theta = pi/4;
internalState.radius = 0.425;
internalState.wheel_base = 0.8;
internalState.gear_ratio = 5;
internalState.steer = 0;
internalState.pedal_speed = 0;

internalState.variance = diag([varX0 varY0 varTheta0 varR0 varB0 varN0 varSteer varPS]);

end


