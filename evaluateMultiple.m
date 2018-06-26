% Written by Professor Mark Mueller, UC Berkeley

%%
% Evaluate a provided solution against many tests.

%%
% Provide the index of the experimental run you would like to use. Note
% that using "0" means that you will load the measurement calibration data.

%%
%==========================================================================
% Here, we run your estimator's initialization
%==========================================================================
clear all;
clc;
%   provide the index of the experimental runs
experimentalRun0 = 100;
experimentalRunf = 120;

totalError = 0;

for experimentalRun = experimentalRun0:(experimentalRunf-1),
	fprintf(['Run = ', num2str(experimentalRun)])
    filename = ['evaluationData/run_' num2str(experimentalRun,'%03d') '.csv'];
	experimentalData = csvread(filename);

	internalState = estInitialize();

	%%
	% Here we will store the estimated position and orientation, for later
	% plotting:

	numDataPoints = size(experimentalData,1);
	estimatedPosition_x = zeros(numDataPoints,1);
	estimatedPosition_y = zeros(numDataPoints,1);
	estimatedAngle = zeros(numDataPoints,1);

	dt = experimentalData(2,1) - experimentalData(1,1);
	for k = 1:numDataPoints
			t = experimentalData(k,1);
			gamma = experimentalData(k,2);
			omega = experimentalData(k,3);
			measx = experimentalData(k,4);
			measy = experimentalData(k,5);
			
			%run the estimator:
			[x, y, theta, internalState] = estRun(t, dt, internalState, gamma, omega, [measx, measy]);

			%keep track:
			estimatedPosition_x(k) = x;
			estimatedPosition_y(k) = y;
			estimatedAngle(k) = theta;
	end    

	% make sure the angle is in [-pi,pi]
	estimatedAngle = mod(estimatedAngle+pi,2*pi)- pi;

	posErr_x = estimatedPosition_x - experimentalData(:,6);
	posErr_y = estimatedPosition_y - experimentalData(:,7);
	angErr   = mod(estimatedAngle - experimentalData(:,8) + pi, 2*pi) - pi;

	ax = sum(abs(posErr_x))/numDataPoints;
	ay = sum(abs(posErr_y))/numDataPoints;
	ath = sum(abs(angErr))/numDataPoints;
	score = ax + ay + ath;

	fprintf(['   pos x = ' num2str(ax) ' m ']);
	fprintf(['   pos y = ' num2str(ay) ' m ']);
	fprintf(['   angle = ' num2str(ath) ' rad ']);
	
	% our scalar score
	fprintf(['Average score: ' num2str(score) ' ']);

	fprintf('\n');

	totalError = totalError + score;
end

fprintf('===========================================\n');
fprintf(['Total score = ',num2str(totalError),'\n'])
