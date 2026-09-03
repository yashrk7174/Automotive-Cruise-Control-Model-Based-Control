function controller = DesignPController(plant, Kp)
%==========================================================================
% DESIGNPCONTROLLER
%==========================================================================
% Project : Professional V2 - Automotive Cruise Control
% File    : DesignPController.m
% Purpose : Design a proportional controller and construct the closed-loop
%           system.
%
% Author  : Yash Khiste
% Version : Professional V2
%
% Inputs:
%   plant : Continuous-time transfer function
%   Kp    : Proportional gain
%
% Outputs:
%   controller : Structure containing controller information
%
%==========================================================================

%% Input Validation

validateattributes(Kp, {'numeric'}, ...
    {'real','scalar','finite','nonnegative'}, ...
    mfilename, 'Kp');

assert(isa(plant,'tf'), ...
    'DesignPController:InvalidPlant', ...
    'Input "plant" must be a transfer function.');

%% Controller Design

controller.Kp = Kp;

controller.C = tf(Kp);

%% Closed-Loop System

controller.OpenLoop = series(controller.C, plant);

controller.ClosedLoop = feedback(controller.OpenLoop, 1);

%% Validation

assert(isstable(controller.ClosedLoop), ...
    'DesignPController:UnstableSystem', ...
    'Closed-loop system is unstable.');

end