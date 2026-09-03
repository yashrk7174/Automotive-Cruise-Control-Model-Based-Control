function GenerateKiStepResponseComparison()

%==========================================================================
% GenerateKiStepResponseComparison
%
% Phase 02 PI Controller Design
%
% Visual comparison of closed-loop response for different Ki values
%
% Fixed:
%       Kp = 500
%
% Tested:
%       Ki = [5 10 20 40 80]
%
% Purpose:
%       Provides engineering evidence for Ki selection
%
%==========================================================================


clc

fprintf('\n=============================================\n');
fprintf(' PI Step Response Comparison\n');
fprintf('=============================================\n');



%% Paths

projectRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));


figureFolder = fullfile( ...
    projectRoot,...
    'Results',...
    'Figures');



%% Plant

params = Parameters();

plant = BuildPlant(params);



%% Controller settings

Kp = 500;

Ki_values = [5 10 20 40 80];



%% Simulation time

t = 0:0.01:100;



%% Plot

figure

hold on



for i = 1:length(Ki_values)


    Ki = Ki_values(i);


    controller = pid(Kp,Ki);


    closedLoop = feedback(controller*plant,1);


    [y,tout] = step(closedLoop,t);


    plot(tout,y,...
        'LineWidth',2,...
        'DisplayName',...
        sprintf('K_i = %d',Ki));


end



grid on


xlabel('Time [s]')

ylabel('Vehicle Speed Response [-]')


title(['Closed-Loop Step Response Comparison ',...
       'for PI Integral Gain Variation'])



legend('Location','southeast')



SaveFigure(gcf,...
    'PI_Ki_StepResponse_Comparison',...
    figureFolder);



fprintf('\nGenerated:\n');

fprintf('PI_Ki_StepResponse_Comparison.png\n');


fprintf('\n=============================================\n');


end