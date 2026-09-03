%%=========================================================================
% MAIN
%==========================================================================
% Project : Professional V2 - Automotive Cruise Control
% File    : Main.m
% Purpose : Execute the complete MATLAB workflow for the P controller.
%
% Author  : Yash khiste
% Version : Professional V2
%==========================================================================

clc;
clear;
close all;

%% Add Utility Functions to Path

currentFolder = fileparts(mfilename('fullpath'));

addpath(fullfile(currentFolder,'Utilities'));

%% Load Parameters

params = Parameters();

%% Build Plant

plant = BuildPlant(params);

%% Design Controller

Kp = 500;

controller = DesignPController(plant, Kp);

%% Step Response

figure('Name','Closed-Loop Step Response');
step(controller.ClosedLoop);
grid on;
title(sprintf('Cruise Control Step Response (Kp = %.0f)', Kp));

%% MATLAB vs Simulink Validation

projectRoot = fileparts(fileparts(mfilename('fullpath')));

modelPath = fullfile( ...
    projectRoot,...
    'Simulink',...
    'CruiseControl_P_Controller.slx');


validation = ValidateSimulink( ...
    controller,...
    modelPath);
%% Performance Metrics

info = stepinfo(controller.ClosedLoop);

disp('----------------------------------------');
disp('Closed-Loop Performance');
disp('----------------------------------------');
disp(info);

%% Closed-Loop Poles

disp('Closed-Loop Poles:');
disp(pole(controller.ClosedLoop));