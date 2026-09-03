
function GeneratePIPerformancePlots()
%==========================================================================
% GeneratePIPerformancePlots
%
% Phase 02 - PI Controller Design
%
% Purpose:
%   Generate final engineering plots for the selected PI controller and
%   compare the final PI response against the established Phase 01
%   P-controller baseline.
%
% Controller:
%   Kp = 500
%   Ki = 20
%
% Figures:
%   1. PI Final Response
%   2. P vs PI Response Comparison
%   3. PI Pole Map
%   4. Steady-State Error Elimination
%
%==========================================================================

clc

fprintf('\n=============================================\n');
fprintf(' Generating PI Performance Plots\n');
fprintf('=============================================\n');


%% Project paths

projectRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));

dataFolder = fullfile( ...
    projectRoot, ...
    'Results', ...
    'Data');

figureFolder = fullfile( ...
    projectRoot, ...
    'Results', ...
    'Figures');


%% Load final PI performance data

piDataFile = fullfile( ...
    dataFolder, ...
    'PI_Controller_Data.mat');

if exist(piDataFile, 'file') ~= 2
    error( ...
        'GeneratePIPerformancePlots:MissingPIData', ...
        ['PI controller data was not found.\n' ...
         'Expected file:\n%s\n\n' ...
         'Run AnalyzePIControllerPerformance first.'], ...
        piDataFile);
end

load(piDataFile, 'PI_results');


%% ============================================================
% 1. Final PI Response
% =============================================================

figure

plot( ...
    PI_results.Time, ...
    PI_results.Response, ...
    'LineWidth', 2)

grid on

xlabel('Time [s]')
ylabel('Vehicle Speed Response [-]')

title([ ...
    'Final PI Controller Closed-Loop Step Response ', ...
    sprintf('(K_p = %.0f, K_i = %.0f)', ...
    PI_results.Kp, ...
    PI_results.Ki)])

ylim([0 1.05])

SaveFigure( ...
    gcf, ...
    'PI_Final_Response', ...
    figureFolder);


%% ============================================================
% 2. P vs PI Response Comparison
% =============================================================

params = Parameters();
plant = BuildPlant(params);

% Use the already selected Kp stored in the PI design data.
% No manual Kp value is introduced here.

P_controller = DesignPController( ...
    plant, ...
    PI_results.Kp);

P_closedLoop = P_controller.ClosedLoop;

% MATLAB step() output order is:
%   [response, time]

[yP, tP] = step( ...
    P_closedLoop, ...
    PI_results.Time);


figure

plot( ...
    tP, ...
    yP, ...
    'LineWidth', 2)

hold on

plot( ...
    PI_results.Time, ...
    PI_results.Response, ...
    'LineWidth', 2)

grid on

xlabel('Time [s]')
ylabel('Vehicle Speed Response [-]')

title('Closed-Loop Response Comparison: P Controller vs PI Controller')

legend( ...
    'P Controller', ...
    'PI Controller', ...
    'Location', ...
    'best')

ylim([0 1.05])

SaveFigure( ...
    gcf, ...
    'P_vs_PI_Response_Comparison', ...
    figureFolder);


%% ============================================================
% 3. PI Pole Map
% =============================================================

figure

plot( ...
    real(PI_results.Poles), ...
    imag(PI_results.Poles), ...
    'rx', ...
    'MarkerSize', 14, ...
    'LineWidth', 3)

grid on

xlabel('Real Axis')
ylabel('Imaginary Axis')

title([ ...
    'PI Controller Closed-Loop Pole Location ', ...
    sprintf('(K_p = %.0f, K_i = %.0f)', ...
    PI_results.Kp, ...
    PI_results.Ki)])

xline(0, 'k--')

SaveFigure( ...
    gcf, ...
    'PI_Pole_Map', ...
    figureFolder);


%% ============================================================
% 4. Steady-State Error Elimination
% =============================================================

figure

reference = ones(size(PI_results.Time));

plot( ...
    PI_results.Time, ...
    reference, ...
    '--', ...
    'LineWidth', 2)

hold on

plot( ...
    tP, ...
    yP, ...
    'LineWidth', 2)

plot( ...
    PI_results.Time, ...
    PI_results.Response, ...
    'LineWidth', 2)

grid on

xlabel('Time [s]')
ylabel('Normalized Vehicle Speed [-]')

title('Steady-State Tracking Performance: Effect of Integral Action')

legend( ...
    'Reference Speed', ...
    'P Controller', ...
    'PI Controller', ...
    'Location', ...
    'best')

ylim([0 1.05])

SaveFigure( ...
    gcf, ...
    'PI_SteadyState_Error_Elimination', ...
    figureFolder);


%% ============================================================
% Completion
% =============================================================

fprintf('\nPI performance plots generated successfully.\n');
fprintf('Figures saved to:\n%s\n', figureFolder);

fprintf('\n=============================================\n');
fprintf(' PI Performance Plot Generation Completed\n');
fprintf('=============================================\n\n');

end
