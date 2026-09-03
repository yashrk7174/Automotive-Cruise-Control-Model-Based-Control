function results = AnalyzeLQRModel()

%==========================================================================
% AnalyzeLQRModel
%
% Project 01 - Automotive Cruise Control - Professional V2
%
% Purpose:
%   Perform the complete baseline LQR engineering analysis:
%
%   1. Build actuator-aware state-space model
%   2. Verify model dimensions
%   3. Analyse open-loop poles
%   4. Analyse controllability
%   5. Define normalized Q/R weighting
%   6. Design baseline LQR controller
%   7. Evaluate closed-loop reference tracking
%   8. Evaluate control effort
%   9. Check actuator constraint
%  10. Save reproducible engineering evidence
%  11. Generate baseline analysis plots
%
% Output:
%   results - Structure containing complete LQR baseline analysis
%
%==========================================================================

%% Initialize project

InitializeProject;

%% Load parameters

params = Parameters();

%% Build actuator-aware LQR plant

sysLQR = BuildLQRPlant(params);

A = sysLQR.A;
B = sysLQR.B;
C = sysLQR.C;
D = sysLQR.D;

%% ========================================================================
% 1. MODEL ANALYSIS
% ========================================================================

numberOfStates  = size(A,1);
numberOfInputs  = size(B,2);
numberOfOutputs = size(C,1);

openLoopPoles = pole(sysLQR);

%% ========================================================================
% 2. CONTROLLABILITY
% ========================================================================

controllabilityMatrix = ctrb(A,B);

controllabilityRank = rank(controllabilityMatrix);

isControllable = ...
    controllabilityRank == numberOfStates;

%% ========================================================================
% 3. NORMALIZED LQR WEIGHTING
%
% Speed state scale:
%   1 m/s
%
% Actuator-force state scale:
%   maximum actuator force
%
% Control input scale:
%   maximum actuator command
% ========================================================================

speedScale = 1;

forceScale = params.maxActuatorForce;

commandScale = params.maxActuatorCommand;

Q = diag([ ...
    1/speedScale^2, ...
    1/forceScale^2]);

R = 1/commandScale^2;

%% ========================================================================
% 4. BASELINE LQR DESIGN
% ========================================================================

[K,~,closedLoopPoles] = lqr(A,B,Q,R);

Acl = A - B*K;

%% ========================================================================
% 5. REFERENCE PRECOMPENSATION
%
% LQR state feedback is a regulator:
%
%       u = -Kx
%
% Nbar provides reference tracking for the 1 m/s cruise-speed command.
% ========================================================================

Nbar = -1/(C*(Acl\B));

sysLQRTracking = ss( ...
    Acl, ...
    B*Nbar, ...
    C, ...
    D);

%% ========================================================================
% 6. CLOSED-LOOP PERFORMANCE
% ========================================================================

simulationTime = (0:0.01:300)';

reference = ones(size(simulationTime));

[y,t,x] = lsim( ...
    sysLQRTracking, ...
    reference, ...
    simulationTime);

info = stepinfo(y,t);

finalSpeed = y(end);

steadyStateError = abs(1-finalSpeed);

%% ========================================================================
% 7. CONTROL EFFORT
% ========================================================================

u = -(x*K.') + Nbar;

controlEffortRMS = rms(u);

controlEffortPeak = max(abs(u));

actuatorLimit = params.maxActuatorCommand;

actuatorConstraintSatisfied = ...
    controlEffortPeak <= actuatorLimit;

%% ========================================================================
% 8. ENGINEERING VERDICT
% ========================================================================

if isControllable && actuatorConstraintSatisfied

    baselineFeasible = true;

else

    baselineFeasible = false;

end

%% ========================================================================
% 9. DISPLAY RESULTS
% ========================================================================

fprintf('\n');
fprintf('============================================================\n');
fprintf('LQR BASELINE ENGINEERING ANALYSIS\n');
fprintf('============================================================\n');

fprintf('\nMODEL\n');
fprintf('States  = %d\n',numberOfStates);
fprintf('Inputs  = %d\n',numberOfInputs);
fprintf('Outputs = %d\n',numberOfOutputs);

fprintf('\nCONTROLLABILITY\n');
fprintf('Rank = %d / %d\n', ...
    controllabilityRank,numberOfStates);

if isControllable
    fprintf('Status = PASS\n');
else
    fprintf('Status = FAIL\n');
end

fprintf('\nLQR WEIGHTING\n');

fprintf('Q =\n');
disp(Q);

fprintf('R =\n');
disp(R);

fprintf('\nLQR GAIN\n');

fprintf('K =\n');
disp(K);

fprintf('Nbar = %.6f\n',Nbar);

fprintf('\nCLOSED-LOOP POLES\n');
disp(closedLoopPoles);

fprintf('\nPERFORMANCE\n');

fprintf('Final Speed      = %.6f m/s\n',finalSpeed);
fprintf('Rise Time        = %.4f s\n',info.RiseTime);
fprintf('Settling Time    = %.4f s\n',info.SettlingTime);
fprintf('Overshoot        = %.4f %%\n',info.Overshoot);
fprintf('SSE              = %.6f\n',steadyStateError);

fprintf('\nCONTROL EFFORT\n');

fprintf('Control RMS      = %.4f N\n',controlEffortRMS);
fprintf('Control Peak     = %.4f N\n',controlEffortPeak);
fprintf('Actuator Limit   = %.4f N\n',actuatorLimit);

if actuatorConstraintSatisfied

    fprintf('Actuator Constraint = PASS\n');

else

    fprintf('Actuator Constraint = FAIL\n');

end

fprintf('\nBASELINE LQR VERDICT\n');

if baselineFeasible

    fprintf('Status = FEASIBLE\n');

else

    fprintf('Status = NOT FEASIBLE\n');

end

fprintf('============================================================\n\n');

%% ========================================================================
% 10. STORE RESULTS
% ========================================================================

results = struct();

% Model
results.system = sysLQR;
results.A = A;
results.B = B;
results.C = C;
results.D = D;

results.numberOfStates = numberOfStates;
results.numberOfInputs = numberOfInputs;
results.numberOfOutputs = numberOfOutputs;

results.openLoopPoles = openLoopPoles;

% Controllability
results.controllabilityMatrix = ...
    controllabilityMatrix;

results.controllabilityRank = ...
    controllabilityRank;

results.isControllable = ...
    isControllable;

% LQR design
results.Q = Q;
results.R = R;

results.speedScale = speedScale;
results.forceScale = forceScale;
results.commandScale = commandScale;

results.K = K;
results.Nbar = Nbar;

results.closedLoopPoles = ...
    closedLoopPoles;

% Tracking system
results.closedLoopSystem = ...
    sysLQRTracking;

results.time = t;
results.reference = reference;
results.speed = y;
results.states = x;
results.controlEffort = u;

% Performance
results.finalSpeed = finalSpeed;
results.riseTime = info.RiseTime;
results.settlingTime = info.SettlingTime;
results.overshoot = info.Overshoot;
results.steadyStateError = ...
    steadyStateError;

% Control effort
results.controlEffortRMS = ...
    controlEffortRMS;

results.controlEffortPeak = ...
    controlEffortPeak;

results.actuatorLimit = ...
    actuatorLimit;

results.actuatorConstraintSatisfied = ...
    actuatorConstraintSatisfied;

results.baselineFeasible = ...
    baselineFeasible;

% Physical parameters
results.mass = params.mass;
results.drag = params.drag;

results.actuatorTimeConstant = ...
    params.actuatorTimeConstant;

%% ========================================================================
% 11. SAVE ENGINEERING DATA
% ========================================================================

analysisFile = mfilename('fullpath');

analysisFolder = fileparts(analysisFile);

matlabRoot = fileparts(analysisFolder);

projectRoot = fileparts(matlabRoot);

dataDir = fullfile( ...
    projectRoot, ...
    'Results', ...
    'Data');

figureDir = fullfile( ...
    projectRoot, ...
    'Results', ...
    'Figures');

if ~exist(dataDir,'dir')
    mkdir(dataDir);
end

if ~exist(figureDir,'dir')
    mkdir(figureDir);
end

dataFile = fullfile( ...
    dataDir, ...
    'LQR_Baseline_Data.mat');

save(dataFile,'-struct','results');

%% ========================================================================
% 12. GENERATE BASELINE RESPONSE PLOT
% ========================================================================

fig = figure('Visible','off');

plot(t,y,'LineWidth',1.5);
hold on;
plot(t,reference,'--','LineWidth',1.2);

grid on;

xlabel('Time [s]');
ylabel('Vehicle Speed [m/s]');

title('LQR Baseline Closed-Loop Response');

legend('LQR Response','Reference','Location','best');

saveas(fig, ...
    fullfile(figureDir,'LQR_Baseline_Response.png'));

close(fig);

%% ========================================================================
% 13. GENERATE CONTROL EFFORT PLOT
% ========================================================================

fig = figure('Visible','off');

plot(t,u,'LineWidth',1.5);
hold on;

yline(actuatorLimit,'--','LineWidth',1.2);
yline(-actuatorLimit,'--','LineWidth',1.2);

grid on;

xlabel('Time [s]');
ylabel('Control Command [N]');

title('LQR Baseline Control Effort');

legend( ...
    'Control Command', ...
    'Actuator Limit', ...
    '-Actuator Limit', ...
    'Location','best');

saveas(fig, ...
    fullfile(figureDir,'LQR_Baseline_ControlEffort.png'));

close(fig);

%% ========================================================================
% 14. GENERATE POLE MAP
% ========================================================================

fig = figure('Visible','off');

plot(real(openLoopPoles), ...
     imag(openLoopPoles), ...
     'x', ...
     'MarkerSize',10, ...
     'LineWidth',2);

hold on;

plot(real(closedLoopPoles), ...
     imag(closedLoopPoles), ...
     'o', ...
     'MarkerSize',8, ...
     'LineWidth',2);

grid on;

xlabel('Real Axis');
ylabel('Imaginary Axis');

title('LQR Open-Loop and Closed-Loop Poles');

legend( ...
    'Open-Loop Poles', ...
    'Closed-Loop Poles', ...
    'Location','best');

saveas(fig, ...
    fullfile(figureDir,'LQR_Baseline_PoleMap.png'));

close(fig);

%% ========================================================================
% 15. FINAL MESSAGE
% ========================================================================

fprintf('LQR baseline data saved:\n');
fprintf('%s\n',dataFile);

fprintf('\nLQR baseline figures saved in:\n');
fprintf('%s\n',figureDir);

fprintf('\n============================================================\n\n');

end