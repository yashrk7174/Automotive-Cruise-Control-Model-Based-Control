%% =========================================================================
% Project      : Automotive Cruise Control
% Phase        : Phase 3 - PID Controller
% File         : AnalyzePIDGainCombinations.m
%
% Description:
% Systematic PID gain-combination analysis.
%
% The script evaluates combinations of:
%
%   Kp = [400 500 600]
%   Ki = [10 20 30]
%   Kd = [0 10 20]
%
% Performance metrics:
%   Rise Time
%   Settling Time
%   Overshoot
%   Steady-State Error
%
% Design requirements:
%   Settling Time < 10 s
%   Overshoot    <= 5 %
%   SSE          < 0.005
%
% =========================================================================

function results = AnalyzePIDGainCombinations()

%% 1. Load plant

params = Parameters();

plant = BuildPlant(params);


%% 2. Define PID gain search space

Kp_values = [400 500 600];

Ki_values = [10 20 30];

Kd_values = [0 10 20];


%% 3. Design requirements

maxSettlingTime = 10;

maxOvershoot = 5;

maxSteadyStateError = 0.005;


%% 4. Number of combinations

numCases = ...
    length(Kp_values) * ...
    length(Ki_values) * ...
    length(Kd_values);


%% 5. Preallocate arrays

Kp_result = zeros(numCases,1);
Ki_result = zeros(numCases,1);
Kd_result = zeros(numCases,1);

RiseTime = zeros(numCases,1);
SettlingTime = zeros(numCases,1);
Overshoot = zeros(numCases,1);
SteadyStateError = zeros(numCases,1);

Stable = false(numCases,1);
RequirementsMet = false(numCases,1);


%% 6. Run gain combinations

caseIndex = 0;

for i = 1:length(Kp_values)

    for j = 1:length(Ki_values)

        for k = 1:length(Kd_values)

            caseIndex = caseIndex + 1;

            %% Current gains

            Kp = Kp_values(i);

            Ki = Ki_values(j);

            Kd = Kd_values(k);


            %% Create PID controller

            controller = DesignPIDController(Kp,Ki,Kd);


            %% Closed-loop system

            closedLoop = feedback(controller.C * plant,1);


            %% Stability check

            poles = pole(closedLoop);

            Stable(caseIndex) = all(real(poles) < 0);


            %% Step response

            [y,t] = step(closedLoop);


            %% Performance metrics

            info = stepinfo(y,t);


            RiseTime(caseIndex) = info.RiseTime;

            SettlingTime(caseIndex) = info.SettlingTime;

            Overshoot(caseIndex) = info.Overshoot;

            SteadyStateError(caseIndex) = abs(1-y(end));


            %% Store gains

            Kp_result(caseIndex) = Kp;

            Ki_result(caseIndex) = Ki;

            Kd_result(caseIndex) = Kd;


            %% Check design requirements

            RequirementsMet(caseIndex) = ...
                Stable(caseIndex) && ...
                SettlingTime(caseIndex) < maxSettlingTime && ...
                Overshoot(caseIndex) <= maxOvershoot && ...
                SteadyStateError(caseIndex) < maxSteadyStateError;

        end

    end

end


%% 7. Create results table

results = table( ...
    Kp_result, ...
    Ki_result, ...
    Kd_result, ...
    RiseTime, ...
    SettlingTime, ...
    Overshoot, ...
    SteadyStateError, ...
    Stable, ...
    RequirementsMet, ...
    'VariableNames', { ...
    'Kp', ...
    'Ki', ...
    'Kd', ...
    'RiseTime', ...
    'SettlingTime', ...
    'Overshoot', ...
    'SteadyStateError', ...
    'Stable', ...
    'RequirementsMet'});


%% 8. Display complete results

fprintf("\n");
fprintf("===============================================================\n");
fprintf("SYSTEMATIC PID GAIN ANALYSIS\n");
fprintf("===============================================================\n");

fprintf("Kp candidates : ");

fprintf("%g ",Kp_values);

fprintf("\n");

fprintf("Ki candidates : ");

fprintf("%g ",Ki_values);

fprintf("\n");

fprintf("Kd candidates : ");

fprintf("%g ",Kd_values);

fprintf("\n");

fprintf("===============================================================\n");

fprintf("DESIGN REQUIREMENTS\n");

fprintf("Settling Time       < %.2f s\n",maxSettlingTime);

fprintf("Overshoot           <= %.2f %%\n",maxOvershoot);

fprintf("Steady-State Error  < %.4f\n",maxSteadyStateError);

fprintf("===============================================================\n\n");


disp(results);


%% 9. Display feasible controllers

feasibleResults = results(results.RequirementsMet,:);


fprintf("\n");
fprintf("===============================================================\n");
fprintf("FEASIBLE PID CONTROLLERS\n");
fprintf("===============================================================\n");

if isempty(feasibleResults)

    fprintf("No controller satisfies all requirements.\n");

else

    disp(feasibleResults);

end


fprintf("===============================================================\n\n");


%% 10. Save results

resultsPath = fullfile( ...
    fileparts(fileparts(mfilename("fullpath"))), ...
    "..", ...
    "Results", ...
    "Data", ...
    "PID_Gain_Search_Data.mat");

resultsPath = string(resultsPath);

save(resultsPath,"results");


fprintf("Results saved to:\n");

fprintf("%s\n",resultsPath);

end