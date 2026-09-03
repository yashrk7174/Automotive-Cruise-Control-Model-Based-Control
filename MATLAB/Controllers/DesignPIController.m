function controller = DesignPIController()
%==========================================================================
% DesignPIController
%
% Phase 02 - PI Controller Design
%
% Creates the final validated PI controller.
%
% Controller Structure:
%
%              Ki
% C(s) = Kp + ----
%               s
%
% Design values obtained from:
%
% 1. Phase 01 P Controller Design
%       Kp = 500
%
% 2. Ki Sensitivity Analysis
%       Ki = 20
%
% Selection criteria:
%
% - Minimum steady-state error
% - Zero overshoot
% - Stable closed-loop poles
% - Acceptable transient response
%
%==========================================================================


%% Final Controller Parameters

Kp = 500;

Ki = 20;



%% Create PI Controller

C = pid(Kp,Ki);



%% Store Controller Information

controller.Type = "PI Controller";

controller.Kp = Kp;

controller.Ki = Ki;

controller.C = C;



%% Engineering Information

controller.DesignMethod = ...
    "Ki Sensitivity Analysis";

controller.SelectionReason = ...
    "Best trade-off between steady-state accuracy, damping and settling performance";



%% Display Confirmation


fprintf('\n=============================================\n');

fprintf(' Final PI Controller Created\n');

fprintf('=============================================\n\n');


fprintf('Controller:\n');

fprintf('             Ki\n');

fprintf('C(s)=Kp + ----\n');

fprintf('              s\n\n');


fprintf('Kp = %.2f\n',Kp);

fprintf('Ki = %.2f\n\n',Ki);


fprintf('Design Method:\n');

fprintf('%s\n',controller.DesignMethod);


fprintf('\nController generation completed.\n');


fprintf('=============================================\n\n');


end