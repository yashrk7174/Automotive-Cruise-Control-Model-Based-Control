function GenerateReport(metrics, validation, controller, outputFolder)
%==========================================================================
% GENERATEREPORT
%==========================================================================
% Project : Professional V2 - Automotive Cruise Control
% File    : GenerateReport.m
% Purpose : Generate engineering report data.
%
% Inputs:
%   metrics     : Performance metrics structure
%   validation  : MATLAB-Simulink validation structure
%   controller  : Controller structure
%   outputFolder: Report destination
%
%==========================================================================


%% Input Validation

assert(isstruct(metrics), ...
    'GenerateReport:InvalidMetrics',...
    'Metrics must be a structure.');

assert(isstruct(validation), ...
    'GenerateReport:InvalidValidation',...
    'Validation must be a structure.');


%% Create Report Folder

if ~exist(outputFolder,'dir')
    mkdir(outputFolder);
end


%% Create Report Data

report.Project = "Automotive Cruise Control - Professional V2";

report.Controller = "P Controller";

report.Kp = controller.Kp;


report.Performance = metrics;


report.Validation.MaxError = validation.MaxError;

report.Validation.RMSE = validation.RMSE;

report.Validation.Pass = validation.Pass;


%% Save Report Data

save(fullfile(outputFolder,...
    'Phase01_Report_Data.mat'),...
    'report');


%% Display Summary

fprintf('\n');
fprintf('====================================\n');
fprintf(' Phase 01 Report Generated\n');
fprintf('====================================\n');

fprintf('Controller : P Controller\n');

fprintf('Kp         : %.2f\n',controller.Kp);

fprintf('Validation : ');

if validation.Pass
    fprintf('PASS\n');
else
    fprintf('FAIL\n');
end

fprintf('====================================\n');


end