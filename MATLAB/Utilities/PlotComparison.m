function PlotComparison(tMATLAB,yMATLAB,tSIM,ySIM)
%==========================================================================
% PLOTCOMPARISON
%==========================================================================
% Project : Professional V2 - Automotive Cruise Control
% File    : PlotComparison.m
% Purpose : Plot MATLAB and Simulink responses for validation.
%
% Inputs:
%   tMATLAB : MATLAB simulation time
%   yMATLAB : MATLAB response
%   tSIM    : Simulink simulation time
%   ySIM    : Simulink response
%
%==========================================================================


%% Input Validation

assert(isnumeric(tMATLAB), ...
    'PlotComparison:InvalidTime', ...
    'MATLAB time vector must be numeric.');

assert(isnumeric(yMATLAB), ...
    'PlotComparison:InvalidResponse', ...
    'MATLAB response must be numeric.');

assert(isnumeric(tSIM), ...
    'PlotComparison:InvalidTime', ...
    'Simulink time vector must be numeric.');

assert(isnumeric(ySIM), ...
    'PlotComparison:InvalidResponse', ...
    'Simulink response must be numeric.');


%% Response Comparison Plot

figure('Name','MATLAB vs Simulink Response');

plot(tMATLAB,yMATLAB,'LineWidth',1.5);
hold on;

plot(tSIM,ySIM,'--','LineWidth',1.5);

grid on;

xlabel('Time (s)');
ylabel('Velocity');

title('MATLAB vs Simulink Closed-Loop Response');

legend('MATLAB','Simulink');


%% Error Plot

errorSignal = interp1( ...
    tSIM,...
    ySIM,...
    tMATLAB,...
    'linear') - yMATLAB;


figure('Name','Validation Error');

plot(tMATLAB,errorSignal,'LineWidth',1.5);

grid on;

xlabel('Time (s)');
ylabel('Error');

title('MATLAB-Simulink Difference');


end