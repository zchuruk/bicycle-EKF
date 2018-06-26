clear all;
close all;
%%
% This is similar to the main function, but rather than running the EKF on
% a single test, it runs it on all test datasets and reports the average
% error and variance of the final position and heading angle

%%
% Provide the number of tests to run
tic
numTests = 99;

% Interchangable for testing various EKFs
%estModel = '8var w/NL noise on theta';
estRunFunc = @estRun;
estInitializeFunc = @estInitialize;

xEndErrors = zeros(numTests,1);
yEndErrors = zeros(numTests,1);
thetaEndErrors = zeros(numTests,1);

for run = 1:numTests
    
    experimentalRun = run;
%     fprintf([num2str(experimentalRun), '.']);
%     if(mod(run,25) == 0)
%         fprintf('\n')
%     end
    filename = ['testData/run_' num2str(experimentalRun,'%03d') '.csv'];
    experimentalData = csvread(filename);
    
    %==========================================================================
    % Here, we run your estimator's initialization
    %==========================================================================
    
    %fprintf('Running the initialization \n');
    internalState = estInitializeFunc();
    P0 = internalState.variance;
    
    
    % Here we will store the estimated position and orientation, for later
    % plotting:
    
    numDataPoints = size(experimentalData,1);
    estimatedPosition_x = zeros(numDataPoints,1);
    estimatedPosition_y = zeros(numDataPoints,1);
    estimatedAngle = zeros(numDataPoints,1);
    
    %My keep track vars
    estVar = cell(numDataPoints,1);
    
    %fprintf('Running the system \n');
    dt = experimentalData(2,1) - experimentalData(1,1);
    for k = 1:numDataPoints
        t = experimentalData(k,1);
        gamma = experimentalData(k,2);
        omega = experimentalData(k,3);
        measx = experimentalData(k,4);
        measy = experimentalData(k,5);
        
        %run the estimator:
        [x, y, theta, internalState] = estRunFunc(t, dt, internalState, gamma, omega, [measx, measy]);
        
        % my keep track
        estVar{k} = internalState.variance;
        
        %keep track:
        estimatedPosition_x(k) = x;
        estimatedPosition_y(k) = y;
        estimatedAngle(k) = theta;
    end
    
    %fprintf('Done running \n');
    % make sure the angle is in [-pi,pi]
    estimatedAngle = mod(estimatedAngle+pi,2*pi)- pi;
    
    posErr_x = estimatedPosition_x - experimentalData(:,6);
    posErr_y = estimatedPosition_y - experimentalData(:,7);
    angErr   = mod(estimatedAngle - experimentalData(:,8) + pi, 2*pi) - pi;
    
%     fprintf('Final error: \n');
%     fprintf(['   pos x = ' num2str(posErr_x(end)) ' m \n']);
%     fprintf(['   pos y = ' num2str(posErr_y(end)) ' m \n']);
%     fprintf(['   angle = ' num2str(angErr(end)) ' rad \n']);
    
    xEndErrors(run) = posErr_x(end);
    yEndErrors(run) = posErr_y(end);
    thetaEndErrors(run) = angErr(end);
    
end
%% Calculate  and print performance statistics
avgXErr = mean(abs(xEndErrors));
avgYErr = mean(abs(yEndErrors));
avgThetaErr = mean(abs(thetaEndErrors));

varXErr = var(abs(xEndErrors));
varYErr = var(abs(yEndErrors));
varThetaErr = var(abs(yEndErrors));

fprintf('\nAverage error: \n');
fprintf(['   pos x = ' num2str(avgXErr) ' m \n']);
fprintf(['   pos y = ' num2str(avgYErr) ' m \n']);
fprintf(['   angle = ' num2str(avgThetaErr) ' rad \n']);
fprintf('Variance of error: \n');
fprintf(['   pos x var = ' num2str(varXErr) '\n']);
fprintf(['   pos y var = ' num2str(varYErr) '\n']);
fprintf(['   angle var = ' num2str(varThetaErr) '\n']);

%% Record Performance
% Q = internalState.Q;
% R = internalState.R;

% if(numTests == 99)
%     
%     load('savedAvgData.mat');
%     savedAvgData.AvgErrors = [savedAvgData.AvgErrors; num2cell(avgXErr), num2cell(avgYErr), num2cell(avgThetaErr)];
%     extracells = [];
%     if(size(internalState.Q,2)==6)
%         extracells = [{NaN},{NaN}];
%     end
%     
%     savedAvgData.P0 = [savedAvgData.P0; num2cell(diag(P0)'),extracells];
%     savedAvgData.Qdiags = [savedAvgData.Qdiags; num2cell(diag(Q)'),extracells];
%     savedAvgData.Rdiags = [savedAvgData.Rdiags; num2cell(diag(R)')];
%     savedAvgData.Model = [savedAvgData.Model; estModel];
%     save('savedAvgData.mat', 'savedAvgData');
% end

toc