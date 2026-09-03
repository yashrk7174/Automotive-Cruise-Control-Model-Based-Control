function results = AnalyzeLQRSensitivity()

%==========================================================================
% AnalyzeLQRSensitivity
%
% Project:
%   Automotive Cruise Control - Professional V2
%
% Purpose:
%   Evaluate the engineering trade-off between state regulation and
%   actuator effort in the LQR controller.
%
% Engineering Question:
%
%   How does the relative weighting between Q and R affect:
%
%       - Tracking performance
%       - Transient response
%       - Control effort
%       - Actuator constraint
%
% Method:
%
%   A baseline Q0/R0 design is defined first.
%   Q and R are then scaled independently around this baseline.
%
%   Important:
%       Scaling Q and R by the same factor does NOT change the optimal
%       LQR controller. Therefore only meaningful relative changes are
%       investigated.
%
% States:
%   x(1) = Vehicle speed [m/s]
%   x(2) = Actuator force [N]
%
% Input:
%   u = Actuator command
%
% Constraint:
%   |u| <= 700 N
%
% Output:
%   results - structure containing all sensitivity results
%
% Saved:
%   Results/Data/LQR_Sensitivity_Data.mat
%   Results/Figures/LQR_Sensitivity_*.png
%
%==========================================================================

%% Initialize project

InitializeProject;

%% Load project parameters

params = Parameters();

%% Build actuator-aware state-space plant

sysLQR = BuildLQRPlant(params);

A = sysLQR.A;
B = sysLQR.B;
C = sysLQR.C;
D = sysLQR.D;

%% Verify controllability

numberOfStates = size(A,1);

controllabilityRank = rank(ctrb(A,B));

if controllabilityRank ~= numberOfStates

    error('LQR plant is not fully controllable.');

end

%% ================================================================
% BASELINE LQR WEIGHTING
% ================================================================

% These are the validated baseline weighting values already used
% during the LQR design stage.

Q0 = diag([1 0]);

R0 = 2.0408e-6;

%% ================================================================
% SENSITIVITY DEFINITION
% ================================================================

% Each case represents a different relative Q/R priority.
%
% alpha = Q scaling
% beta  = R scaling
%
% The important quantity is the relative weighting alpha/beta.

cases = {

    "Low State Priority",       0.5,  1.0
    "Baseline",                 1.0,  1.0
    "Higher State Priority",    2.0,  1.0
    "Higher Control Penalty",   1.0,  2.0
    "Strong Control Penalty",   1.0,  5.0
    "Balanced Higher Weight",   2.0,  2.0

    };

numberOfCases = size(cases,1);

%% Preallocate

caseName = strings(numberOfCases,1);

alpha = zeros(numberOfCases,1);
beta  = zeros(numberOfCases,1);

relativeWeight = zeros(numberOfCases,1);

riseTime = nan(numberOfCases,1);
settlingTime = nan(numberOfCases,1);
overshoot = nan(numberOfCases,1);
steadyStateError = nan(numberOfCases,1);

controlRMS = nan(numberOfCases,1);
controlPeak = nan(numberOfCases,1);

stable = false(numberOfCases,1);
actuatorConstraintMet = false(numberOfCases,1);

KValues = zeros(numberOfCases,numberOfStates);
closedLoopPoles = zeros(numberOfCases,numberOfStates);

QValues = zeros(numberOfCases,numberOfStates);
RValues = zeros(numberOfCases,1);

%% Simulation time

t = (0:0.01:300)';

reference = ones(size(t));

%% ================================================================
% RUN LQR SENSITIVITY STUDY
% ================================================================

for i = 1:numberOfCases

    %% Case information

    caseName(i) = cases{i,1};

    alpha(i) = cases{i,2};

    beta(i) = cases{i,3};

    relativeWeight(i) = alpha(i)/beta(i);

    %% Scaled Q and R

    Q = alpha(i)*Q0;

    R = beta(i)*R0;

    QValues(i,:) = diag(Q).';

    RValues(i) = R;

    %% LQR controller

    [K,~,~] = lqr(A,B,Q,R);

    KValues(i,:) = K;

    %% Closed-loop system

    Acl = A - B*K;

    closedLoopPoles(i,:) = eig(Acl);

    stable(i) = all(real(closedLoopPoles(i,:)) < 0);

    %% Reference scaling

    Nbar = -1/(C*(Acl\B));

    %% Tracking system

    sysTracking = ss( ...
        Acl, ...
        B*Nbar, ...
        C, ...
        D);

    %% Output response

    [y,~,x] = lsim( ...
        sysTracking, ...
        reference, ...
        t);

    %% Performance metrics

    info = stepinfo(y,t,1);

    riseTime(i) = info.RiseTime;

    settlingTime(i) = info.SettlingTime;

    overshoot(i) = info.Overshoot;

    steadyStateError(i) = abs(1-y(end));

    %% Control command

    u = -(x*K.') + Nbar;

    controlRMS(i) = rms(u);

    controlPeak(i) = max(abs(u));

    %% Actuator constraint

    actuatorConstraintMet(i) = ...
        controlPeak(i) <= params.maxActuatorCommand;

end

%% ================================================================
% DISPLAY RESULTS
% ================================================================

fprintf('\n');

fprintf('============================================================\n');
fprintf('LQR Q/R RELATIVE WEIGHTING SENSITIVITY\n');
fprintf('============================================================\n');

fprintf('Baseline Q =\n');
disp(Q0);

fprintf('Baseline R = %.6e\n',R0);

fprintf('Actuator limit = %.4f N\n', ...
    params.maxActuatorCommand);

fprintf('============================================================\n\n');

fprintf('%-25s %-8s %-8s %-10s %-11s %-11s %-11s %-14s %-14s %-10s\n', ...
    'Case', ...
    'alpha', ...
    'beta', ...
    'Q/R', ...
    'RiseTime', ...
    'Settling', ...
    'Overshoot', ...
    'SSE', ...
    'ControlPeak', ...
    'Feasible');

fprintf('%s\n',repmat('-',1,145));

for i = 1:numberOfCases

    fprintf('%-25s %-8.2f %-8.2f %-10.3f %-11.4f %-11.4f %-11.4f %-14.6f %-14.4f %-10s\n', ...
        caseName(i), ...
        alpha(i), ...
        beta(i), ...
        relativeWeight(i), ...
        riseTime(i), ...
        settlingTime(i), ...
        overshoot(i), ...
        steadyStateError(i), ...
        controlPeak(i), ...
        string(actuatorConstraintMet(i)));

end

fprintf('\n============================================================\n');

%% ================================================================
% IDENTIFY FEASIBLE DESIGNS
% ================================================================

feasible = ...
    stable & ...
    actuatorConstraintMet;

fprintf('FEASIBLE LQR CONTROLLERS\n');
fprintf('============================================================\n');

for i = 1:numberOfCases

    if feasible(i)

        fprintf('%-25s  Rise = %.4f s   Settling = %.4f s   Peak = %.4f N\n', ...
            caseName(i), ...
            riseTime(i), ...
            settlingTime(i), ...
            controlPeak(i));

    end

end

fprintf('============================================================\n\n');

%% ================================================================
% STORE RESULTS
% ================================================================

results = struct();

results.caseName = caseName;

results.alpha = alpha;

results.beta = beta;

results.relativeWeight = relativeWeight;

results.QBaseline = Q0;

results.RBaseline = R0;

results.QValues = QValues;

results.RValues = RValues;

results.K = KValues;

results.closedLoopPoles = closedLoopPoles;

results.riseTime = riseTime;

results.settlingTime = settlingTime;

results.overshoot = overshoot;

results.steadyStateError = steadyStateError;

results.controlRMS = controlRMS;

results.controlPeak = controlPeak;

results.stable = stable;

results.actuatorConstraintMet = ...
    actuatorConstraintMet;

results.feasible = feasible;

results.controllabilityRank = ...
    controllabilityRank;

results.numberOfStates = ...
    numberOfStates;

results.mass = params.mass;

results.drag = params.drag;

results.actuatorTimeConstant = ...
    params.actuatorTimeConstant;

results.actuatorLimit = ...
    params.maxActuatorCommand;

%% ================================================================
% SAVE DATA
% ================================================================

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

save( ...
    fullfile(dataDir,'LQR_Sensitivity_Data.mat'), ...
    '-struct','results');

%% ================================================================
% FIGURE 1 - SETTLING TIME
% ================================================================

figure;

plot(relativeWeight, ...
     settlingTime, ...
     'o-', ...
     'LineWidth',1.5);

grid on;

xlabel('Relative Q/R Weighting');

ylabel('Settling Time [s]');

title('LQR Sensitivity - Settling Time');

saveas(gcf, ...
    fullfile( ...
    figureDir, ...
    'LQR_Sensitivity_SettlingTime.png'));

%% ================================================================
% FIGURE 2 - CONTROL PEAK
% ================================================================

figure;

plot(relativeWeight, ...
     controlPeak, ...
     'o-', ...
     'LineWidth',1.5);

hold on;

yline( ...
    params.maxActuatorCommand, ...
    '--', ...
    'Actuator Limit');

grid on;

xlabel('Relative Q/R Weighting');

ylabel('Peak Control Command [N]');

title('LQR Sensitivity - Actuator Demand');

saveas(gcf, ...
    fullfile( ...
    figureDir, ...
    'LQR_Sensitivity_ControlPeak.png'));

%% ================================================================
% FIGURE 3 - CONTROL RMS
% ================================================================

figure;

plot(relativeWeight, ...
     controlRMS, ...
     'o-', ...
     'LineWidth',1.5);

grid on;

xlabel('Relative Q/R Weighting');

ylabel('Control RMS [N]');

title('LQR Sensitivity - Control Effort');

saveas(gcf, ...
    fullfile( ...
    figureDir, ...
    'LQR_Sensitivity_ControlRMS.png'));

%% ================================================================
% FINAL MESSAGE
% ================================================================

fprintf('\nLQR sensitivity data saved:\n');

fprintf('%s\n', ...
    fullfile( ...
    dataDir, ...
    'LQR_Sensitivity_Data.mat'));

fprintf('\nLQR sensitivity figures saved in:\n');

fprintf('%s\n',figureDir);

fprintf('\n============================================================\n');
fprintf('LQR SENSITIVITY ANALYSIS COMPLETE\n');
fprintf('============================================================\n\n');

end
