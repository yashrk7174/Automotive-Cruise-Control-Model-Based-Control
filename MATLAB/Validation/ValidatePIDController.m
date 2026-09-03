function validation = ValidatePIDController()
% ValidatePIDController
% ---------------------------------------------------------
% Project 01: Automotive Cruise Control
% Professional V2
%
% Purpose:
%   Cross-validate the filtered PID controller implemented
%   analytically in MATLAB against the Simulink model.
%
% Controller:
%   Kp = 500
%   Ki = 30
%   Kd = 20
%   N  = 10
%
% Validation:
%   - Response comparison
%   - Maximum absolute error
%   - RMSE
%   - Final-value comparison
%   - Performance-metric comparison
%   - Validation PASS / FAIL
% ---------------------------------------------------------

fprintf('\n');
fprintf('============================================================\n');
fprintf('PID MATLAB <-> SIMULINK VALIDATION\n');
fprintf('============================================================\n');

%% 1. Initialize project


params = Parameters();

%% 2. Build plant

plant = BuildPlant(params);

%% 3. Build filtered PID controller

Kp = 500;
Ki = 30;
Kd = 20;
N  = 10;

controller = DesignFilteredPIDController(Kp,Ki,Kd,N);

C = controller.C;

%% 4. MATLAB closed-loop model

closedLoop = feedback(C * plant,1);

fprintf('\nMATLAB controller:\n');
fprintf('Kp = %.2f\n',Kp);
fprintf('Ki = %.2f\n',Ki);
fprintf('Kd = %.2f\n',Kd);
fprintf('N  = %.2f\n',N);

%% 5. MATLAB response

simulationTime = 300;

fprintf('\nGenerating MATLAB response...\n');

t_matlab = linspace(0,simulationTime,30001);

[y_matlab,t_matlab] = step(closedLoop,t_matlab);

y_matlab = squeeze(y_matlab);
t_matlab = squeeze(t_matlab);

%% 6. Run Simulink

modelName = 'CruiseControl_PID_Controller';

fprintf('Running Simulink model...\n');

load_system(modelName);

out = sim(modelName);

fprintf('Simulation completed.\n');

%% 7. Extract Simulink data

simSignal = out.sim_speed;

t_sim = simSignal.time;
y_sim = simSignal.signals.values;

t_sim = t_sim(:);
y_sim = y_sim(:);

%% 8. Interpolate MATLAB response onto Simulink time vector

y_matlab_interp = interp1( ...
    t_matlab, ...
    y_matlab, ...
    t_sim, ...
    'linear', ...
    'extrap');

%% 9. Response error

difference = y_matlab_interp - y_sim;

maxError = max(abs(difference));

rmse = sqrt(mean(difference.^2));

finalMATLAB = y_matlab_interp(end);
finalSimulink = y_sim(end);

finalDifference = abs(finalMATLAB-finalSimulink);

%% 10. Performance metrics

matlabInfo = stepinfo(y_matlab,t_matlab);

simulinkInfo = stepinfo(y_sim,t_sim);

%% 11. Validation tolerances

responseTolerance = 1e-4;
finalTolerance = 1e-5;

%% 12. Validation decision

responsePass = maxError <= responseTolerance;
finalPass = finalDifference <= finalTolerance;

validationPass = responsePass && finalPass;

%% 13. Display results

fprintf('\n');
fprintf('============================================================\n');
fprintf('PID VALIDATION RESULTS\n');
fprintf('============================================================\n');

fprintf('\nResponse comparison:\n');

fprintf('Maximum Absolute Error = %.10e\n',maxError);
fprintf('RMSE                   = %.10e\n',rmse);

fprintf('\nFinal value:\n');

fprintf('MATLAB Final Speed     = %.10f\n',finalMATLAB);
fprintf('Simulink Final Speed   = %.10f\n',finalSimulink);
fprintf('Final Difference       = %.10e\n',finalDifference);

fprintf('\nPerformance comparison:\n');

fprintf('%-20s %-15s %-15s\n', ...
    'Metric','MATLAB','Simulink');

fprintf('%-20s %-15.6f %-15.6f\n', ...
    'Rise Time', ...
    matlabInfo.RiseTime, ...
    simulinkInfo.RiseTime);

fprintf('%-20s %-15.6f %-15.6f\n', ...
    'Settling Time', ...
    matlabInfo.SettlingTime, ...
    simulinkInfo.SettlingTime);

fprintf('%-20s %-15.6f %-15.6f\n', ...
    'Overshoot', ...
    matlabInfo.Overshoot, ...
    simulinkInfo.Overshoot);

fprintf('\n');

if validationPass
    fprintf('============================================================\n');
    fprintf('VALIDATION : PASS\n');
    fprintf('============================================================\n');
else
    fprintf('============================================================\n');
    fprintf('VALIDATION : FAIL\n');
    fprintf('============================================================\n');
end

%% 14. Generate comparison figure

figure('Name','PID MATLAB vs Simulink');

plot(t_matlab,y_matlab,'LineWidth',1.5);
hold on;

plot(t_sim,y_sim,'--','LineWidth',1.5);

grid on;

xlabel('Time (s)');
ylabel('Vehicle Speed (m/s)');

title('PID Controller: MATLAB vs Simulink');

legend('MATLAB','Simulink','Location','southeast');

figDir = fullfile( ...
    fileparts(fileparts(mfilename('fullpath'))), ...
    '..', 'Results', 'Figures');

figDir = char(java.io.File(figDir).getCanonicalPath());

if ~exist(figDir,'dir')
    mkdir(figDir);
end

saveas(gcf, ...
    fullfile(figDir,'PID_MATLAB_vs_Simulink.png'));
%% 15. Plot error

figure('Name','PID Validation Error');

plot(t_sim,difference,'LineWidth',1.5);

grid on;

xlabel('Time (s)');
ylabel('MATLAB - Simulink');

title('PID MATLAB-Simulink Validation Error');

saveas(gcf, ...
    fullfile(figDir,'PID_Validation_Error.png'));
%% 16. Store validation results

validation.Controller.Kp = Kp;
validation.Controller.Ki = Ki;
validation.Controller.Kd = Kd;
validation.Controller.N  = N;

validation.Response.MaxError = maxError;
validation.Response.RMSE = rmse;

validation.FinalValue.MATLAB = finalMATLAB;
validation.FinalValue.Simulink = finalSimulink;
validation.FinalValue.Difference = finalDifference;

validation.Metrics.MATLAB = matlabInfo;
validation.Metrics.Simulink = simulinkInfo;

validation.Tolerance.Response = responseTolerance;
validation.Tolerance.FinalValue = finalTolerance;

validation.Pass.Response = responsePass;
validation.Pass.FinalValue = finalPass;
validation.Pass.Overall = validationPass;

fprintf('\nValidation data stored in variable: validation\n');




%% 17. Save validation data

resultsDir = fullfile( ...
    fileparts(fileparts(mfilename('fullpath'))), ...
    '..', 'Results', 'Data');

resultsDir = char(java.io.File(resultsDir).getCanonicalPath());

if ~exist(resultsDir,'dir')
    mkdir(resultsDir);
end

save(fullfile(resultsDir,'PID_Validation_Data.mat'), ...
    'validation');

fprintf('\nPID validation data saved:\n');
fprintf('%s\n', ...
    fullfile(resultsDir,'PID_Validation_Data.mat'));

end