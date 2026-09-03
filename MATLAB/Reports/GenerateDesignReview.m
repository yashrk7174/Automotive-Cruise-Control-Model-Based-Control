function GenerateDesignReview()

%==========================================================================
% GenerateDesignReview
%
% Phase 01 Professional Design Review Generator
%
% Creates:
%   Results/Reports/Phase01_DesignReview.tex
%   Results/Reports/Phase01_DesignReview.pdf
%
%==========================================================================


clc

fprintf('\n');
fprintf('=============================================\n');
fprintf(' Phase 01 Design Review Generator\n');
fprintf('=============================================\n');


%% Project Root

projectRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));


%% Results Paths

dataFolder = fullfile( ...
    projectRoot,...
    'Results',...
    'Data');


figureFolder = fullfile( ...
    projectRoot,...
    'Results',...
    'Figures');


reportFolder = fullfile( ...
    projectRoot,...
    'Results',...
    'Reports');


if ~exist(reportFolder,'dir')
    mkdir(reportFolder);
end


%% Load Data


controllerFile = fullfile(dataFolder,...
    'Controller_Data.mat');

validationFile = fullfile(dataFolder,...
    'Validation_Data.mat');

sensitivityFile = fullfile(dataFolder,...
    'Kp_Sensitivity_Data.mat');

effortFile = fullfile(dataFolder,...
    'ControlEffort_Data.mat');


assert(isfile(controllerFile),...
    'Controller_Data.mat missing');


assert(isfile(validationFile),...
    'Validation_Data.mat missing');


assert(isfile(sensitivityFile),...
    'Kp_Sensitivity_Data.mat missing');


assert(isfile(effortFile),...
    'ControlEffort_Data.mat missing');



load(controllerFile);

load(validationFile);

load(sensitivityFile,'results');

sensitivityResults = results;

load(effortFile,'results');

effortResults = results;


%% Rebuild Controller Object

params = Parameters();

plant = BuildPlant(params);

controller = DesignPController(plant,Kp);


%% Create Latex File


latexFile = fullfile( ...
    reportFolder,...
    'Phase01_DesignReview.tex');



BuildLatexDocument( ...
    latexFile,...
    projectRoot,...
    controller,...
    validation,...
    sensitivityResults,...
    figureFolder);


%% Compile PDF


CompileLatexPDF( ...
    latexFile,...
    reportFolder);



fprintf('\n');
fprintf('=============================================\n');
fprintf(' Phase 01 Report Completed\n');
fprintf('=============================================\n');


end