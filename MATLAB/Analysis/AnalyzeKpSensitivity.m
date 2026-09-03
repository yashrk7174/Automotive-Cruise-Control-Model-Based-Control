function results = AnalyzeKpSensitivity()
%==========================================================================
% AnalyzeKpSensitivity
%
% Purpose:
%   Perform proportional gain sensitivity analysis for the Cruise Control
%   System by evaluating closed-loop performance over a range of Kp values.
%
% Inputs:
%   None
%
% Outputs:
%   results : Structure array containing controller performance metrics
%
% Generated File:
%   Results/Data/Kp_Sensitivity_Data.mat
%
% Author:
%   Professional V2
%==========================================================================

%% Initialization

clc;

fprintf('\n');
fprintf('=============================================\n');
fprintf(' P Controller Gain Sensitivity Analysis\n');
fprintf('=============================================\n');

%% Load Plant

params = Parameters();
plant  = BuildPlant(params);

%% Analysis Parameters

KpValues = [50 100 200 500 1000 2000];

nCases = numel(KpValues);

results = struct([]);

%% Run Analysis

for k = 1:nCases

    Kp = KpValues(k);

    controller = DesignPController(plant, Kp);

    sys = controller.ClosedLoop;

    info = stepinfo(sys);

    dcGain = dcgain(sys);

    steadyStateError = abs(1 - dcGain);

    poles = pole(sys);

    results(k).Kp               = Kp;
    results(k).RiseTime         = info.RiseTime;
    results(k).SettlingTime     = info.SettlingTime;
    results(k).Overshoot        = info.Overshoot;
    results(k).Peak             = info.Peak;
    results(k).PeakTime         = info.PeakTime;
    results(k).SteadyStateError = steadyStateError;
    results(k).Pole             = poles;

end

%% Display Summary

T = struct2table(results);

disp(T)

%% Save Results

projectRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));

dataFolder = fullfile( ...
    projectRoot, ...
    'Results', ...
    'Data');

if ~exist(dataFolder,'dir')
    mkdir(dataFolder);
end

save( ...
    fullfile(dataFolder,'Kp_Sensitivity_Data.mat'), ...
    'results');

fprintf('\n');
fprintf('Sensitivity data saved successfully.\n');
fprintf('%s\n',fullfile(dataFolder,'Kp_Sensitivity_Data.mat'));

fprintf('=============================================\n');

end