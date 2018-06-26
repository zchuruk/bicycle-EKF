function [x,y,theta,internalStateOut] = estRun(time, dt, internalStateIn, steeringAngle, pedalSpeed, measurement)
% In this function we implement an estimator with the following inputs:
%  time: current time in [s]
%  dt: current time step [s]
%  internalStateIn: the estimator internal state, definition up to you.
%  steeringAngle: the steering angle of the bike, gamma, [rad]
%  pedalSpeed: the rotational speed of the pedal, omega, [rad/s]
%  measurement: the position measurement valid at the current time step
%
% Note: the measurement is a 2D vector, of x-y position measurement.
%  The measurement sensor may fail to return data, in which case the
%  measurement is given as NaN.
%
% The function has four outputs:
%  est_x: current best estimate for the bicycle's x-position
%  est_y: current best estimate for the bicycle's y-position
%  est_theta: current best estimate for the bicycle's rotation theta
%  internalState: the estimator's internal state, in the same format as
%  when it is passed in

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%% Define Noise Variances %%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Process Variance terms
Qx = .0033;
Qy = .0033;
Qtheta = 8.3333e-04;
Qr = 0;
QB = 0;
QN = 0.0169;
Qsteer = 2.0833e-06;
Qps = 8.3333e-04;
Q = diag([Qx Qy Qtheta Qr QB QN Qsteer Qps]);

% Measurement variance terms
Rx = 1.089;
Ry = 2.988;
R = diag([Rx Ry]);

%% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Run EKF %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Prior Update
[zp, Pp] = LOCALpriorUpdate(internalStateIn, steeringAngle, pedalSpeed, Q, dt);

if ~isnan(measurement(1)) & ~isnan(measurement(2))
    % have a valid measurement
    [zm, Pm] = LOCALmeasUpdate(zp, Pp, R, measurement);
    
    state = zm;
    variance = Pm;
else
    state = zp;
    variance = Pp;
end

%% OUTPUTS %%
% Update the internal state (will be passed as an argument to the function
% at next run)
x = state(1);
y = state(2);
theta = state(3);

internalStateOut.x = state(1);
internalStateOut.y = state(2);
internalStateOut.theta = state(3);
internalStateOut.radius = state(4);
internalStateOut.wheel_base = state(5);
internalStateOut.gear_ratio = state(6);
internalStateOut.steer = state(7);
internalStateOut.pedal_speed = state(8);
internalStateOut.variance = variance;

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%% Local Helper Functions %%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [zp, Pp] = LOCALpriorUpdate(internalState, steeringAngle, pedalSpeed, Q, dt)
% Performs the EKF prior update, returning the new state and variance

x = internalState.x;               
y = internalState.y;
theta = internalState.theta;
r = internalState.radius;
B = internalState.wheel_base;
N = internalState.gear_ratio;
gamma = steeringAngle;
omega = pedalSpeed;

Pm = internalState.variance;

zp = [x + dt*N*omega*r*cos(theta);
      y + dt*N*omega*r*sin(theta);
      theta + dt*(N*omega*r/B)*tan(gamma);
      r;
      B;
      N;
      gamma;
      omega];

% Jacobian calculated offline using predefined continuous dynamics 
% equations (not included here) with the following code:
%%% f = [x_dot; y_dot; theta_dot; r_dot; B_dot; N_dot; gamma_dot; omega_dot];
%%% dfdx = jacobian(f, [x,y,theta,r,B,N, gamma, omega]);
%%% dfdv = jacobian(f, [v1,v2,v3,v4,v5,v6,v7,v8]);
v3 = 0;
dfdx = [ 0, 0, -N*omega*r*sin(theta),        N*omega*cos(theta),                              0,        omega*r*cos(theta),                                   0,        N*r*cos(theta);
         0, 0,  N*omega*r*cos(theta),        N*omega*sin(theta),                              0,        omega*r*sin(theta),                                   0,        N*r*sin(theta);
         0, 0,                     0, (1+v3)*N*omega*tan(gamma)/B, -(1+v3)*N*omega*r*tan(gamma)/B^2, (1+v3)*omega*r*tan(gamma)/B, (1+v3)*N*omega*r*(tan(gamma)^2 + 1)/B, (1+v3)*N*r*tan(gamma)/B;
         0, 0,                     0,                         0,                              0,                         0,                                   0,                     0;
         0, 0,                     0,                         0,                              0,                         0,                                   0,                     0;
         0, 0,                     0,                         0,                              0,                         0,                                   0,                     0;
         0, 0,                     0,                         0,                              0,                         0,                                   0,                     0;
         0, 0,                     0,                         0,                              0,                         0,                                   0,                     0];

dfdv = [ 0, 0,                        0, 0, 0, 0, 0, 0;
         0, 0,                        0, 0, 0, 0, 0, 0;
         0, 0, (N*omega*r*tan(gamma))/B, 0, 0, 0, 0, 0;
         0, 0,                        0, 0, 0, 0, 0, 0;
         0, 0,                        0, 0, 0, 0, 0, 0;
         0, 0,                        0, 0, 0, 0, 0, 0;
         0, 0,                        0, 0, 0, 0, 0, 0;
         0, 0,                        0, 0, 0, 0, 0, 0];

% Discretize A and L
A = eye(size(dfdx)) + dfdx*dt;

% Note: do not add 1 to the theta term of L when discretizing b/c there 
% is no additive noise term in that equation
L = diag([1,1,0,1,1,1,1,1]) + dfdv*dt;

% Pp = A*Pm*A' + L*internalState.Q*L';
Pp = A*Pm*A' + L*Q*L';
end

function [zm, Pm] = LOCALmeasUpdate(zp, Pp, R, measurement)
% Performs the EKF measurement update, returning the new state and variance

x = zp(1);
y = zp(2);
theta = zp(3);
% r = zp(4);
B = zp(5);
% N = zp(6);
% gamma = zp(7);
% omega = zp(8);

measModel = [x + 1/2*B*cos(theta);
             y + 1/2*B*sin(theta)];

% Jacobian calculated offline from the measurement equations using the
% following code:
%%% h = [p1; p2];
%%% dhdx = jacobian(h, [x,y,theta,r,B,N, gamma, omega]);
dhdx = [ 1, 0, -(B*sin(theta))/2, 0, cos(theta)/2, 0, 0, 0
         0, 1,  (B*cos(theta))/2, 0, sin(theta)/2, 0, 0, 0];

H = dhdx;
M = eye(2); % Only additive measurement noise
measurement = reshape(measurement,[2,1]);

K = Pp*H'*inv(H*Pp*H' + M*R*M');
zm = zp + K*(measurement - measModel);
Pm = (eye(size(Pp)) - K*H)*Pp;

end