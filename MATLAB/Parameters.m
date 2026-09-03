function params = Parameters()
%==========================================================================
% PARAMETERS
%==========================================================================
% Project : Professional V2 - Automotive Cruise Control
% File    : Parameters.m
% Purpose : Define and validate all physical and project parameters.
%
% Author  : Yash Khiste
% Version : Professional V2
%
%
% Outputs:
%   params : Structure containing vehicle, environment, and project data.
%
% Dependencies:
%   None
%
%==========================================================================

%% Project Information
params.projectName = "Automotive Cruise Control";
params.version     = "Professional V2";
%params.author      = "Yash";

%% Vehicle Parameters
params.mass = 1000;          % Vehicle mass [kg]
params.drag = 50;            % Viscous damping coefficient [N*s/m]
params.actuatorTimeConstant = 0.5;   % Actuator time constant [s]
params.maxActuatorForce   = 700;    % Maximum actuator force [N]
params.maxActuatorCommand = 700;    % Maximum actuator command [N]

%% Environmental Parameters
params.gravity = 9.81;       % Gravitational acceleration [m/s^2]

%% Parameter Validation

assert(isnumeric(params.mass) && isscalar(params.mass) && params.mass > 0, ...
    'Parameters:InvalidMass', ...
    'Vehicle mass must be a positive scalar.');

assert(isnumeric(params.drag) && isscalar(params.drag) && params.drag >= 0, ...
    'Parameters:InvalidDrag', ...
    'Drag coefficient must be a non-negative scalar.');

assert(isnumeric(params.gravity) && isscalar(params.gravity) && params.gravity > 0, ...
    'Parameters:InvalidGravity', ...
    'Gravity must be a positive scalar.');
assert(isnumeric(params.actuatorTimeConstant) && ...
    isscalar(params.actuatorTimeConstant) && ...
    params.actuatorTimeConstant > 0, ...
    'Parameters:InvalidActuatorTimeConstant', ...
    'Actuator time constant must be a positive scalar.');

end