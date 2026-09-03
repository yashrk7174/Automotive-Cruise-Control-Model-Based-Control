function results = AnalyzeControlEffort()
%==========================================================================
% AnalyzeControlEffort
%
% Purpose:
%   Evaluate actuator/control effort required for different P controller
%   gains in the cruise control system.
%
% Engineering Objective:
%   Study the trade-off between faster response and actuator demand.
%
% Inputs:
%   None
%
% Outputs:
%   results:
%       Kp
%       Time
%       ControlSignal
%       MaximumEffort
%       RMSEffort
%
% Saved Data:
%   Results/Data/ControlEffort_Data.mat
%
%==========================================================================


%% Initialization

clc;

fprintf('\n');
fprintf('=============================================\n');
fprintf(' Control Effort Analysis\n');
fprintf('=============================================\n');


%% Build Plant

params = Parameters();

plant = BuildPlant(params);


%% Kp Values

Kp_values = [
    50
    100
    200
    500
    1000
    2000
];


%% Simulation Time

t = 0:0.01:60;


%% Storage

results = struct([]);


%% Analysis Loop


for i = 1:length(Kp_values)


    Kp = Kp_values(i);


    controller = DesignPController(plant,Kp);


    closedLoop = controller.ClosedLoop;


    %% Closed loop response

    [y,tout] = step(closedLoop,t);


    %% Error signal

    reference = ones(size(tout));

    error = reference - y;


    %% Controller output

    u = Kp .* error;



    results(i).Kp = Kp;

    results(i).Time = tout;

    results(i).ControlSignal = u;

    results(i).MaximumEffort = max(abs(u));

    results(i).RMSEffort = sqrt(mean(u.^2));


end



%% Display Results


fprintf('\n');
fprintf('Kp       Maximum Effort       RMS Effort\n');
fprintf('------------------------------------------\n');


for i = 1:length(results)

fprintf('%4d       %.4f              %.4f\n',...
    results(i).Kp,...
    results(i).MaximumEffort,...
    results(i).RMSEffort);

end



%% Save Data


projectRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));


dataFolder = fullfile(...
    projectRoot,...
    'Results',...
    'Data');


if ~exist(dataFolder,'dir')
    mkdir(dataFolder);
end


save(...
    fullfile(dataFolder,...
    'ControlEffort_Data.mat'),...
    'results');



fprintf('\nControl effort data saved successfully.\n');

fprintf('%s\n',...
    fullfile(dataFolder,...
    'ControlEffort_Data.mat'));

fprintf('=============================================\n');


end