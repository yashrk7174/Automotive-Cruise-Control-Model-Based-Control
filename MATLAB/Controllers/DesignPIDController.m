%% =========================================================================
% Project      : Automotive Cruise Control
% Phase        : Phase 3 - PID Controller
% File         : DesignPIDController.m
%
% Description:
% Creates the PID controller used for the cruise-control system.
%
% Controller:
% C(s) = Kp + Ki/s + Kd*s
%
% =========================================================================

function controller = DesignPIDController(Kp, Ki, Kd)

%% Input validation

if ~isscalar(Kp) || ~isnumeric(Kp) || Kp < 0
    error("Kp must be a non-negative numeric scalar.");
end

if ~isscalar(Ki) || ~isnumeric(Ki) || Ki < 0
    error("Ki must be a non-negative numeric scalar.");
end

if ~isscalar(Kd) || ~isnumeric(Kd) || Kd < 0
    error("Kd must be a non-negative numeric scalar.");
end


%% Create PID controller

C = pid(Kp, Ki, Kd);


%% Store controller information

controller.Kp = Kp;
controller.Ki = Ki;
controller.Kd = Kd;

controller.C = C;


%% Display design information

fprintf("\nPID Controller\n");
fprintf("-------------------------\n");
fprintf("Kp = %.4f\n", Kp);
fprintf("Ki = %.4f\n", Ki);
fprintf("Kd = %.4f\n", Kd);
fprintf("-------------------------\n");

end