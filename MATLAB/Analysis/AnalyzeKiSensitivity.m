function AnalyzeKiSensitivity()

%==========================================================================
% AnalyzeKiSensitivity
%
% Phase 02 - PI Controller Design
%
% Purpose:
%   Evaluate integral gain influence while keeping Kp fixed.
%
% Methodology:
%
%   Kp is selected from Phase 01:
%       Kp = 500
%
%   Ki is varied and performance is evaluated.
%
% Metrics:
%   - Rise Time
%   - Settling Time
%   - Overshoot
%   - Steady State Error
%   - Pole Location
%
% Output:
%   Results/Data/Ki_Sensitivity_Data.mat
%
%==========================================================================


clc

fprintf('\n=============================================\n');
fprintf(' Phase 02 - Ki Sensitivity Analysis\n');
fprintf(' PI Controller Design\n');
fprintf('=============================================\n\n');


%% Project Path

projectRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));


dataFolder = fullfile( ...
    projectRoot,...
    'Results',...
    'Data');



%% Load Plant

params = Parameters();

plant = BuildPlant(params);



%% Fixed Kp from Phase 01

Kp = 500;


fprintf('Fixed Proportional Gain:\n');

fprintf('Kp = %.2f\n\n',Kp);



%% Integral Gain Variation

% Engineering selected range
% Ki is increased gradually to observe effect

Ki_values = [
    5
    10
    20
    40
    80
];



fprintf('Testing Ki values:\n');

disp(Ki_values)



%% Simulation Time

t = 0:0.01:100;



%% Preallocate

results = struct();



%% Ki Loop

for i = 1:length(Ki_values)


    Ki = Ki_values(i);


    %% PI Controller

    C = pid(Kp,Ki);


    %% Closed Loop

    sys = feedback(C*plant,1);



    %% Response

    [response,time] = step(sys,t);



    %% Metrics

    info = stepinfo(response,time);



    steadyStateError = abs(1-response(end));


    poles = pole(sys);



    %% Store Results


    results(i).Kp = Kp;

    results(i).Ki = Ki;

    results(i).RiseTime = info.RiseTime;

    results(i).SettlingTime = info.SettlingTime;

    results(i).Overshoot = info.Overshoot;

    results(i).Peak = info.Peak;

    results(i).PeakTime = info.PeakTime;

    results(i).SteadyStateError = steadyStateError;

    results(i).Poles = poles;



    %% Display


    fprintf('\nKi = %.2f\n',Ki);

    fprintf('Rise Time          = %.4f s\n',info.RiseTime);

    fprintf('Settling Time      = %.4f s\n',info.SettlingTime);

    fprintf('Overshoot          = %.2f %%\n',info.Overshoot);

    fprintf('Steady State Error = %.6f\n',steadyStateError);

    fprintf('Poles:\n');

    disp(poles)



end



%% Save Data


if ~isfolder(dataFolder)

    mkdir(dataFolder)

end



save(...
    fullfile(dataFolder,...
    'Ki_Sensitivity_Data.mat'),...
    'results',...
    'Kp',...
    'Ki_values',...
    'params',...
    'plant');



fprintf('\n=============================================\n');

fprintf('Ki Sensitivity Analysis Completed\n');

fprintf('Data Saved:\n');

fprintf('%s\n',...
    fullfile(dataFolder,...
    'Ki_Sensitivity_Data.mat'));

fprintf('=============================================\n');



end