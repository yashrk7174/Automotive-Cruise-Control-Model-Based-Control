function validation = ValidateSimulink(controller, modelPath)
%==========================================================================
% VALIDATESIMULINK
%==========================================================================
% Project : Professional V2 - Automotive Cruise Control
% File    : ValidateSimulink.m
% Purpose : Compare MATLAB closed-loop response with Simulink simulation.
%
% Author  : Yash Khiste
% Version : Professional V2
% 
%
% Inputs:
%   controller : Controller structure from DesignPController.m
%   modelPath  : Full path of Simulink model
%
% Outputs:
%   validation : Structure containing validation results
%
%==========================================================================


%% Input Validation

assert(isstruct(controller), ...
    'ValidateSimulink:InvalidController', ...
    'Controller input must be a structure.');

assert(isfield(controller,'ClosedLoop'), ...
    'ValidateSimulink:MissingClosedLoop', ...
    'Controller structure must contain ClosedLoop.');

assert(isa(controller.ClosedLoop,'tf'), ...
    'ValidateSimulink:InvalidPlant', ...
    'ClosedLoop must be a transfer function.');

assert(isfile(modelPath), ...
    'ValidateSimulink:ModelMissing', ...
    'Simulink model file does not exist: %s', modelPath);



%% Simulink Simulation

load_system(modelPath);

simData = sim(modelPath);

%% MATLAB Response

simulationTime = 0:0.01:20;

[yMATLAB,tMATLAB] = step( ...
    controller.ClosedLoop,...
    simulationTime);


%% Extract Simulink Output

simSignal = simData.simOut;

tSIM = simSignal.time;

ySIM = simSignal.signals.values;

%% Response Alignment

ySIM_interp = interp1( ...
    tSIM,...
    ySIM,...
    tMATLAB,...
    'linear');


%% Error Calculation

errorSignal = yMATLAB - ySIM_interp;


validation.MaxError = max(abs(errorSignal));

validation.RMSE = sqrt(mean(errorSignal.^2));


%% Performance Comparison

validation.MATLAB = stepinfo( ...
    controller.ClosedLoop);

validation.Simulink.RiseTime = ...
    stepinfo(ySIM_interp,tMATLAB).RiseTime;


%% Pass / Fail Decision

validation.Tolerance = 1e-5;

validation.Pass = ...
    validation.MaxError < validation.Tolerance;


%% Display Report

fprintf('\n');
fprintf('========================================\n');
fprintf(' MATLAB vs SIMULINK VALIDATION\n');
fprintf('========================================\n');

fprintf('Maximum Error : %.3e\n', ...
    validation.MaxError);

fprintf('RMSE          : %.3e\n', ...
    validation.RMSE);


if validation.Pass
    fprintf('\nResult : PASS\n');
else
    fprintf('\nResult : FAIL\n');
end


fprintf('========================================\n');


end