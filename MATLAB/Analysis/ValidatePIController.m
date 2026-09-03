function validation = ValidatePIController(controller, modelPath)
%==========================================================================
% ValidatePIController
%
% Compares MATLAB analytical PI controller response with Simulink response.
%
% Validation Metrics:
%   - Maximum Absolute Error
%   - Root Mean Square Error (RMSE)
%   - PASS / FAIL Decision
%
% Professional Version 2
%==========================================================================

fprintf('\n');
fprintf('=============================================\n');
fprintf(' PI Controller Validation\n');
fprintf('=============================================\n');

%% Input Validation

assert(isstruct(controller), ...
    'controller must be a structure.');

assert(isfield(controller,'C'), ...
    'Controller structure must contain field C.');

assert(isfile(modelPath), ...
    'Simulink model not found:\n%s', modelPath);

%% Build Plant

params = Parameters();

plant = BuildPlant(params);

%% Closed-loop MATLAB Model

closedLoop = feedback(controller.C * plant, 1);

%% Run Simulink Model

fprintf('\nRunning Simulink model...\n');

out = sim(modelPath);

%% Extract Simulink Data

simData = out.simulink_response;

tSim = simData.time;
ySim = simData.signals.values;

%% MATLAB Analytical Response

fprintf('Running MATLAB analytical model...\n');

closedLoop = feedback(controller.C * plant, 1);

% Use the actual Simulink simulation time
tValidation = out.tout;

[yMatlab, tMatlab] = step(closedLoop, tValidation(end));

% Interpolate MATLAB response onto Simulink time vector
yMatlabInterp = interp1( ...
    tMatlab, ...
    yMatlab, ...
    tValidation, ...
    'linear');

% Define common validation time vector
tValidation = linspace(0,100,2001)';

[yMatlab,~] = step(closedLoop,tValidation);


%% Interpolate Simulink response

ySimInterp = interp1( ...
    tSim,...
    ySim,...
    tValidation,...
    'linear',...
    'extrap');
if any(isnan(ySimInterp))

    error('Simulink response contains invalid values');

end
%% Error Calculation

error = yMatlab - ySimInterp;

maxError = max(abs(error));

rmse = sqrt(mean(error.^2));

%% Validation Criteria

tolerance = 1e-5;

pass = maxError < tolerance;

%% Create Validation Structure

validation = struct();

validation.MaxError = maxError;
validation.RMSE = rmse;
validation.Tolerance = tolerance;
validation.Pass = pass;

validation.MATLAB.Time = tValidation;
validation.MATLAB.Response = yMatlab;

validation.Simulink.Time = tValidation;
validation.Simulink.Response = ySimInterp;

validation.Error = error;

%% Console Output

fprintf('\n');
fprintf('Maximum Error : %.6e\n', maxError);
fprintf('RMSE          : %.6e\n', rmse);
fprintf('Tolerance     : %.6e\n', tolerance);

if pass
    fprintf('\nValidation Result : PASS\n');
else
    fprintf('\nValidation Result : FAIL\n');
end

fprintf('=============================================\n');

end