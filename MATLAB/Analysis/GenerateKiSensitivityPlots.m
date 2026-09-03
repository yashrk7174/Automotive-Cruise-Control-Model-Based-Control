function GenerateKiSensitivityPlots()

%==========================================================================
% GenerateKiSensitivityPlots
%
% Phase 02 - PI Controller Design
%
% Purpose:
%   Visual engineering analysis of integral gain influence.
%
% Fixed:
%       Kp = 500
%
% Variable:
%       Ki sensitivity
%
% Selected:
%       Ki = 20
%
% Generated figures support PI controller design decision.
%
%==========================================================================


clc

fprintf('\n=============================================\n');
fprintf(' Generating PI Controller Ki Sensitivity Plots\n');
fprintf('=============================================\n');


%% Project paths

projectRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));


dataFolder = fullfile( ...
    projectRoot,...
    'Results',...
    'Data');


figureFolder = fullfile( ...
    projectRoot,...
    'Results',...
    'Figures');



%% Load sensitivity data

load(fullfile(dataFolder,...
    'Ki_Sensitivity_Data.mat'));



%% Extract results

Ki = [results.Ki];

RiseTime = [results.RiseTime];

SettlingTime = [results.SettlingTime];

Overshoot = [results.Overshoot];

SteadyStateError = [results.SteadyStateError];


selectedKi = 20;



%% Helper for selected point

selectedIndex = find(Ki == selectedKi);



%% ============================================================
% 1. Transient Speed Analysis
% =============================================================

figure

plot(Ki,RiseTime,'-o',...
    'LineWidth',2)

hold on

plot(selectedKi,...
    RiseTime(selectedIndex),...
    'ro',...
    'MarkerSize',10,...
    'LineWidth',2)


grid on


xlabel('Integral Gain K_i')

ylabel('Rise Time [s]')


title(['PI Controller Transient Response Speed Sensitivity ',...
       'to Integral Gain Variation'])


legend(...
    'Tested Integral Gain Values',...
    'Selected Design Point (K_i = 20)',...
    'Location','best')


SaveFigure(gcf,...
    'PI_TransientSpeed_Analysis',...
    figureFolder);



%% ============================================================
% 2. Settling Performance Analysis
% =============================================================

figure


plot(Ki,SettlingTime,'-o',...
    'LineWidth',2)

hold on


plot(selectedKi,...
    SettlingTime(selectedIndex),...
    'ro',...
    'MarkerSize',10,...
    'LineWidth',2)


grid on


xlabel('Integral Gain K_i')

ylabel('Settling Time [s]')


title(['Closed-Loop Settling Performance Sensitivity ',...
       'to Integral Gain Variation'])


legend(...
    'Tested Integral Gain Values',...
    'Selected Design Point (K_i = 20)',...
    'Location','best')


SaveFigure(gcf,...
    'PI_SettlingPerformance_Analysis',...
    figureFolder);



%% ============================================================
% 3. Damping Behaviour Analysis
% =============================================================

figure


plot(Ki,Overshoot,'-o',...
    'LineWidth',2)


hold on


plot(selectedKi,...
    Overshoot(selectedIndex),...
    'ro',...
    'MarkerSize',10,...
    'LineWidth',2)


grid on


xlabel('Integral Gain K_i')

ylabel('Maximum Overshoot [%]')


title(['Closed-Loop Damping Behaviour Under ',...
       'Increasing Integral Action'])


legend(...
    'Overshoot Response',...
    'Selected Design Point (K_i = 20)',...
    'Location','best')


SaveFigure(gcf,...
    'PI_Damping_Analysis',...
    figureFolder);



%% ============================================================
% 4. Tracking Accuracy Analysis
% =============================================================

figure


semilogy(Ki,SteadyStateError,'-o',...
    'LineWidth',2)


hold on


plot(selectedKi,...
    SteadyStateError(selectedIndex),...
    'ro',...
    'MarkerSize',10,...
    'LineWidth',2)


grid on


xlabel('Integral Gain K_i')

ylabel('Steady-State Error [-]')


title(['Steady-State Tracking Accuracy Improvement ',...
       'with Integral Action'])


legend(...
    'Tracking Error',...
    'Selected Design Point (K_i = 20)',...
    'Location','best')


SaveFigure(gcf,...
    'PI_TrackingAccuracy_Analysis',...
    figureFolder);



%% ============================================================
% 5. Pole Migration Analysis
% =============================================================

figure

hold on


for i = 1:length(results)


    poles = results(i).Poles;


    plot(real(poles),...
         imag(poles),...
         'x',...
         'MarkerSize',12,...
         'LineWidth',2)


    for j = 1:length(poles)

        text(real(poles(j))+0.015,...
             imag(poles(j)),...
             sprintf('Ki=%d',results(i).Ki))

    end


end


grid on


xlabel('Real Axis')

ylabel('Imaginary Axis')


title(['Closed-Loop Pole Migration ',...
       'with Increasing Integral Gain'])


SaveFigure(gcf,...
    'PI_PoleMigration_Analysis',...
    figureFolder);



%% ============================================================
% 6. Final Design Trade-off Summary
% =============================================================

figure


plot(Ki,RiseTime,'-o',...
    'LineWidth',2)

hold on

plot(Ki,SettlingTime,'-s',...
    'LineWidth',2)


plot(selectedKi,...
    SettlingTime(selectedIndex),...
    'ro',...
    'MarkerSize',10,...
    'LineWidth',2)


grid on


xlabel('Integral Gain K_i')

ylabel('Time [s]')


title(['PI Controller Design Trade-off Analysis: ',...
       'Integral Gain Selection'])


legend(...
    'Rise Time',...
    'Settling Time',...
    'Selected Design Point (K_i = 20)',...
    'Location','best')


SaveFigure(gcf,...
    'PI_GainSelection_Tradeoff',...
    figureFolder);



fprintf('\nFigures generated successfully.\n');

fprintf('Selected PI controller:\n');

fprintf('Kp = 500\n');

fprintf('Ki = 20\n');


fprintf('\n=============================================\n');


end