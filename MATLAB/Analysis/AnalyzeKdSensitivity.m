%% =========================================================================
% Project      : Automotive Cruise Control
% Phase        : Phase 3 - PID Controller
% File         : AnalyzeKdSensitivity.m
%
% Description:
% Performs derivative-gain sensitivity analysis for the PID controller.
%
% Fixed gains:
%   Kp = 500
%   Ki = 20
%
% Variable gain:
%   Kd = [0 5 10 20 40 80]
%
% Performance metrics:
%   Rise Time
%   Settling Time
%   Overshoot
%   Steady-State Error
%   Control Effort RMS
%   Control Effort Peak
%
% Note:
% The ideal derivative term is evaluated numerically in the time domain.
% This avoids attempting to simulate an improper ideal PID transfer
% function directly. A filtered derivative will be introduced later
% for industrial implementation.
%
% =========================================================================

function results = AnalyzeKdSensitivity()

%% 1. Load plant

params = Parameters();

plant = BuildPlant(params);


%% 2. Controller parameters

Kp = 500;
Ki = 20;

Kd_values = [0 5 10 20 40 80];


%% 3. Preallocate result arrays

numCases = length(Kd_values);

RiseTime           = zeros(numCases,1);
SettlingTime       = zeros(numCases,1);
Overshoot          = zeros(numCases,1);
SteadyStateError   = zeros(numCases,1);
ControlEffort_RMS  = zeros(numCases,1);
ControlEffort_Peak = zeros(numCases,1);


%% 4. Sensitivity analysis

for i = 1:numCases

    Kd = Kd_values(i);

    %% Create PID controller

    controller = DesignPIDController(Kp,Ki,Kd);


    %% Closed-loop system

    closedLoop = feedback(controller.C * plant,1);


    %% Vehicle response

    [y,t] = step(closedLoop);


    %% Performance metrics

    info = stepinfo(y,t);

    RiseTime(i) = info.RiseTime;

    SettlingTime(i) = info.SettlingTime;

    Overshoot(i) = info.Overshoot;

    SteadyStateError(i) = abs(1-y(end));


    %% Error signal

    reference = ones(size(y));

    errorSignal = reference - y;


    %% Integral term

    integralError = cumtrapz(t,errorSignal);


    %% Derivative term

    derivativeError = gradient(errorSignal,t);


    %% PID control effort

    u = Kp .* errorSignal ...
        + Ki .* integralError ...
        + Kd .* derivativeError;


    %% Control effort metrics

    ControlEffort_RMS(i) = sqrt(mean(u.^2));

    ControlEffort_Peak(i) = max(abs(u));

end


%% 5. Create results table

results = table( ...
    Kd_values(:), ...
    RiseTime, ...
    SettlingTime, ...
    Overshoot, ...
    SteadyStateError, ...
    ControlEffort_RMS, ...
    ControlEffort_Peak, ...
    'VariableNames', { ...
    'Kd', ...
    'RiseTime', ...
    'SettlingTime', ...
    'Overshoot', ...
    'SteadyStateError', ...
    'ControlEffort_RMS', ...
    'ControlEffort_Peak'});


%% 6. Display results

fprintf("\n");
fprintf("===============================================================\n");
fprintf("PID DERIVATIVE GAIN SENSITIVITY ANALYSIS\n");
fprintf("===============================================================\n");

fprintf("Fixed Kp = %.2f\n",Kp);
fprintf("Fixed Ki = %.2f\n",Ki);

fprintf("===============================================================\n\n");

disp(results);

end