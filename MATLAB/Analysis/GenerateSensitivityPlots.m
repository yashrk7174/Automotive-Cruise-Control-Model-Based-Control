function GenerateSensitivityPlots()
%==========================================================================
% GenerateSensitivityPlots
%
% Purpose:
%   Generate engineering plots from Kp sensitivity analysis.
%
% Input:
%   Results/Data/Kp_Sensitivity_Data.mat
%
% Output:
%   Figures saved to Results/Figures
%
%==========================================================================

%% Initialization

clc;

fprintf('\n');
fprintf('=============================================\n');
fprintf(' Generating Sensitivity Plots\n');
fprintf('=============================================\n');

%% Locate Project

projectRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));

dataFile = fullfile( ...
    projectRoot,...
    'Results',...
    'Data',...
    'Kp_Sensitivity_Data.mat');

assert(isfile(dataFile),...
    'Run AnalyzeKpSensitivity first.');

load(dataFile,'results');

%% Figure Folder

figureFolder = fullfile( ...
    projectRoot,...
    'Results',...
    'Figures');

if ~exist(figureFolder,'dir')
    mkdir(figureFolder);
end

%% Extract Data

Kp = [results.Kp];
RiseTime = [results.RiseTime];
SettlingTime = [results.SettlingTime];
Overshoot = [results.Overshoot];
SSE = [results.SteadyStateError];
Pole = real([results.Pole]);

%% Plot 1

figure

plot(Kp,RiseTime,'-o','LineWidth',2)

grid on

xlabel('K_p')

ylabel('Rise Time (s)')

title('Rise Time vs K_p')

SaveFigure(gcf,'RiseTime_vs_Kp',figureFolder)

%% Plot 2

figure

plot(Kp,SettlingTime,'-o','LineWidth',2)

grid on

xlabel('K_p')

ylabel('Settling Time (s)')

title('Settling Time vs K_p')

SaveFigure(gcf,'SettlingTime_vs_Kp',figureFolder)

%% Plot 3

figure

plot(Kp,Overshoot,'-o','LineWidth',2)

grid on

xlabel('K_p')

ylabel('Overshoot (%)')

title('Overshoot vs K_p')

SaveFigure(gcf,'Overshoot_vs_Kp',figureFolder)

%% Plot 4

figure

plot(Kp,SSE,'-o','LineWidth',2)

grid on

xlabel('K_p')

ylabel('Steady-State Error')

title('Steady-State Error vs K_p')

SaveFigure(gcf,'SteadyStateError_vs_Kp',figureFolder)

%% Pole Map

figure

hold on


for i = 1:length(Pole)

    plot(Pole(i),0,...
        'x',...
        'MarkerSize',12,...
        'LineWidth',2);

    text(Pole(i),0,...
        sprintf('Kp=%d',Kp(i)),...
        'VerticalAlignment','bottom');

end


grid on

xlabel('Real Axis')

ylabel('Imaginary Axis')

title('Closed-Loop Pole Movement with Kp')


hold off


SaveFigure(gcf,'Pole_Map',figureFolder)

%% Plot 6

figure

hold on

params = Parameters();

plant = BuildPlant(params);

colors = lines(length(Kp));

for i = 1:length(Kp)

    controller = DesignPController(plant,Kp(i));

    step(controller.ClosedLoop,'Color',colors(i,:));

end

grid on

legend(compose('K_p = %d',Kp),...
    'Location','southeast')

title('Closed-Loop Step Response Comparison')

xlabel('Time (s)')

ylabel('Vehicle Speed')

SaveFigure(gcf,'Kp_Comparison',figureFolder)

%% Performance Summary Dashboard

figure


subplot(2,2,1)

plot(Kp,RiseTime,'-o','LineWidth',2)

grid on

xlabel('K_p')

ylabel('Rise Time (s)')

title('Rise Time')


subplot(2,2,2)

plot(Kp,SettlingTime,'-o','LineWidth',2)

grid on

xlabel('K_p')

ylabel('Settling Time (s)')

title('Settling Time')


subplot(2,2,3)

plot(Kp,Overshoot,'-o','LineWidth',2)

grid on

xlabel('K_p')

ylabel('Overshoot (%)')

title('Overshoot')


subplot(2,2,4)

plot(Kp,SSE,'-o','LineWidth',2)

grid on

xlabel('K_p')

ylabel('Steady-State Error')

title('Steady-State Error')


sgtitle('P Controller Performance Summary - Kp Sensitivity')


SaveFigure(gcf,...
    'Performance_Summary',...
    figureFolder)


fprintf('\nAll plots generated successfully.\n');
fprintf('=============================================\n');

end