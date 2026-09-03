function GenerateControlEffortPlots()
%==========================================================================
% GenerateControlEffortPlots
%
% Purpose:
%   Generate actuator/control effort plots from saved sensitivity data.
%
% Input:
%   Results/Data/ControlEffort_Data.mat
%
% Output:
%   Results/Figures/
%
%==========================================================================


%% Initialization

clc;


fprintf('\n');
fprintf('=============================================\n');
fprintf(' Generating Control Effort Plots\n');
fprintf('=============================================\n');



%% Locate Project

projectRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));


dataFile = fullfile(...
    projectRoot,...
    'Results',...
    'Data',...
    'ControlEffort_Data.mat');


assert(isfile(dataFile),...
    'Control effort data file missing. Run AnalyzeControlEffort first.');



%% Load Data

load(dataFile,'results');



%% Figure Folder

figureFolder = fullfile(...
    projectRoot,...
    'Results',...
    'Figures');


if ~exist(figureFolder,'dir')
    mkdir(figureFolder);
end



%% Extract Data

Kp = [results.Kp];

maxEffort = [results.MaximumEffort];

rmsEffort = [results.RMSEffort];



%% Plot 1
% Control signal response for different gains


figure

hold on


for i = 1:length(results)

    plot(...
        results(i).Time,...
        results(i).ControlSignal,...
        'LineWidth',1.5);

end


grid on


xlabel('Time (s)')

ylabel('Actuator Command (Normalized)')


title('Actuator Effort Response for Different K_p Values')


legend(...
    compose('K_p = %d',Kp),...
    'Location',...
    'best')


hold off


SaveFigure(...
    gcf,...
    'Control_Effort',...
    figureFolder);



%% Plot 2
% Peak and RMS effort comparison


figure


plot(Kp,maxEffort,'-o',...
    'LineWidth',2)


hold on


plot(Kp,rmsEffort,'-s',...
    'LineWidth',2)


grid on


xlabel('K_p')

ylabel('Actuator Command (Normalized)')

title('Actuator Effort vs K_p')


legend(...
    'Maximum Effort',...
    'RMS Effort',...
    'Location',...
    'best')


hold off


SaveFigure(...
    gcf,...
    'ControlEffort_vs_Kp',...
    figureFolder);



fprintf('\n');

fprintf('Control effort plots generated successfully.\n');


fprintf('=============================================\n');


end