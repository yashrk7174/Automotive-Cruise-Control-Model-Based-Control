function results = AnalyzeDerivativeFilterSensitivity()

%% ================================================================
% AnalyzeDerivativeFilterSensitivity
%
% Purpose:
% Evaluate the effect of derivative filter coefficient N on
% practical PID controller performance.
%
% Fixed PID gains:
%
% Kp = 500
% Ki = 30
% Kd = 20
%
% Tested filter coefficients:
%
% N = [1 2 5 10 20 50 100]
%
% Metrics:
%   Rise Time
%   Settling Time
%   Overshoot
%   Steady-State Error
%   Control Effort RMS
%   Control Effort Peak
%
% ================================================================


%% 1. Load plant

params = Parameters();

plant = BuildPlant(params);


%% 2. Fixed PID gains

Kp = 500;
Ki = 30;
Kd = 20;


%% 3. Filter coefficients

N_values = [1 2 5 10 20 50 100];


%% 4. Simulation time

t = 0:0.1:300;


%% 5. Preallocate

n = length(N_values);

RiseTime = zeros(n,1);
SettlingTime = zeros(n,1);
Overshoot = zeros(n,1);

SteadyStateError = zeros(n,1);

ControlEffortRMS = zeros(n,1);
ControlEffortPeak = zeros(n,1);

Stable = false(n,1);


%% 6. Evaluate each filter coefficient

for i = 1:n

    N = N_values(i);


    %% Create filtered PID

    controller = ...
        DesignFilteredPIDController(Kp,Ki,Kd,N);


    %% Closed-loop reference -> output

    closedLoop = ...
        feedback(controller.C * plant,1);


    %% Stability

    poles = pole(closedLoop);

    Stable(i) = all(real(poles) < 0);


    %% Output response

    [y,t_response] = step(closedLoop,t);


    %% Performance metrics

    info = stepinfo(y,t_response);


    RiseTime(i) = info.RiseTime;

    SettlingTime(i) = info.SettlingTime;

    Overshoot(i) = info.Overshoot;


    %% Steady-state error

    SteadyStateError(i) = ...
        abs(1 - dcgain(closedLoop));


    %% Control effort transfer function

    controlTransfer = ...
        feedback(controller.C,plant);


    %% Control response

    [u,tu] = step(controlTransfer,t);


    %% Control metrics

    ControlEffortRMS(i) = rms(u);

    ControlEffortPeak(i) = max(abs(u));

end


%% 7. Create results table

results = table( ...
    N_values(:), ...
    RiseTime, ...
    SettlingTime, ...
    Overshoot, ...
    SteadyStateError, ...
    ControlEffortRMS, ...
    ControlEffortPeak, ...
    Stable, ...
    'VariableNames', { ...
    'N', ...
    'RiseTime', ...
    'SettlingTime', ...
    'Overshoot', ...
    'SteadyStateError', ...
    'ControlEffortRMS', ...
    'ControlEffortPeak', ...
    'Stable'});


%% 8. Display results

fprintf("\n");
fprintf("===============================================================\n");
fprintf("DERIVATIVE FILTER SENSITIVITY ANALYSIS\n");
fprintf("===============================================================\n");

fprintf("Fixed Kp = %.2f\n",Kp);
fprintf("Fixed Ki = %.2f\n",Ki);
fprintf("Fixed Kd = %.2f\n",Kd);

fprintf("===============================================================\n\n");

disp(results);


%% 9. Save results

projectRoot = ...
    fileparts(fileparts(mfilename("fullpath")));

resultsPath = fullfile( ...
    projectRoot, ...
    "..", ...
    "Results", ...
    "Data", ...
    "Derivative_Filter_Sensitivity_Data.mat");


save(resultsPath,"results");


fprintf("\nResults saved to:\n%s\n\n",resultsPath);

end