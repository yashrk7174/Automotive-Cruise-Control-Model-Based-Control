function AnalyzePIControllerPerformance()

%==========================================================================
% AnalyzePIControllerPerformance
%
% Phase 02 - PI Controller Design
%
% Purpose:
%   Evaluate final PI controller performance after Ki selection.
%
% Controller:
%
%              Ki
% C(s) = Kp + ----
%               s
%
% Selected:
%
%       Kp = 500
%       Ki = 20
%
% Outputs:
%
%       Rise Time
%       Settling Time
%       Overshoot
%       Peak Value
%       Peak Time
%       Steady State Error
%       Closed Loop Poles
%
% Data saved:
%
%       Results/Data/PI_Controller_Data.mat
%
%==========================================================================


clc

fprintf('\n=============================================\n');
fprintf(' PI Controller Performance Analysis\n');
fprintf('=============================================\n\n');



%% Project paths

projectRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));


dataFolder = fullfile( ...
    projectRoot,...
    'Results',...
    'Data');



%% Load plant

params = Parameters();

plant = BuildPlant(params);



%% Load final controller

controller = DesignPIController();



%% Closed loop system

closedLoop = feedback( ...
    controller.C * plant,...
    1);



%% Step response

time = 0:0.01:100;


[response,time] = step(closedLoop,time);



%% Performance metrics

metrics = stepinfo(response,time);



%% Steady state error

steadyStateValue = dcgain(closedLoop);


steadyStateError = abs(1-steadyStateValue);



%% Poles

closedLoopPoles = pole(closedLoop);



%% Stability check

if all(real(closedLoopPoles)<0)

    stability = "Stable";

else

    stability = "Unstable";

end



%% Store results


PI_results.Controller = controller.Type;

PI_results.Kp = controller.Kp;

PI_results.Ki = controller.Ki;


PI_results.Time = time;

PI_results.Response = response;


PI_results.RiseTime = metrics.RiseTime;

PI_results.SettlingTime = metrics.SettlingTime;

PI_results.Overshoot = metrics.Overshoot;

PI_results.Peak = metrics.Peak;

PI_results.PeakTime = metrics.PeakTime;


PI_results.SteadyStateError = steadyStateError;


PI_results.Poles = closedLoopPoles;


PI_results.Stability = stability;


PI_results.ClosedLoop = closedLoop;



%% Display engineering summary


fprintf('Final PI Controller:\n\n');


fprintf('Kp = %.2f\n',controller.Kp);

fprintf('Ki = %.2f\n\n',controller.Ki);



fprintf('Performance Results:\n');

fprintf('Rise Time          = %.4f s\n', ...
    PI_results.RiseTime);


fprintf('Settling Time      = %.4f s\n',...
    PI_results.SettlingTime);


fprintf('Overshoot          = %.2f %%\n',...
    PI_results.Overshoot);


fprintf('Peak Value         = %.4f\n',...
    PI_results.Peak);


fprintf('Peak Time          = %.4f s\n',...
    PI_results.PeakTime);


fprintf('Steady State Error = %.6e\n',...
    PI_results.SteadyStateError);



fprintf('\nClosed Loop Poles:\n');

disp(PI_results.Poles)



fprintf('System Status: %s\n',...
    PI_results.Stability);



%% Save data


save(fullfile(dataFolder,...
    'PI_Controller_Data.mat'),...
    'PI_results',...
    'controller',...
    'closedLoop');



fprintf('\nData Saved:\n');

fprintf('%s\n',...
fullfile(dataFolder,...
'PI_Controller_Data.mat'));



fprintf('\n=============================================\n');

fprintf(' PI Performance Analysis Completed\n');

fprintf('=============================================\n');


end