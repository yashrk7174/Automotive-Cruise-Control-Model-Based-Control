function metrics = ComputeMetrics(system)
%==========================================================================
% COMPUTEMETRICS
%==========================================================================
% Project : Professional V2 - Automotive Cruise Control
% File    : ComputeMetrics.m
% Purpose : Calculate closed-loop performance metrics.
%
% Author  : Yash
% Version : Professional V2
% Inputs:
%   system : Closed-loop transfer function
%
% Outputs:
%   metrics : Structure containing performance parameters
%
%==========================================================================


%% Input Validation

assert(isa(system,'tf'), ...
    'ComputeMetrics:InvalidSystem', ...
    'Input must be a transfer function.');


%% Time Response Metrics

info = stepinfo(system);


metrics.RiseTime = info.RiseTime;

metrics.SettlingTime = info.SettlingTime;

metrics.Overshoot = info.Overshoot;

metrics.PeakTime = info.PeakTime;

metrics.PeakValue = info.Peak;


%% Steady-State Analysis

dcGain = dcgain(system);

metrics.SteadyStateValue = dcGain;

metrics.SteadyStateError = abs(1-dcGain);


%% Stability Analysis

poles = pole(system);

metrics.Poles = poles;

metrics.Stable = all(real(poles)<0);


%% Store Summary

metrics.SystemOrder = order(system);


end