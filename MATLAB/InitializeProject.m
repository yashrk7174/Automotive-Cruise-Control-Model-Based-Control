function InitializeProject()
%==========================================================================
% InitializeProject
%
% Professional V2 - Project Initialization
%
% Purpose:
%   Establish a deterministic project environment independent of the
%   current MATLAB working directory.
%
%==========================================================================

fprintf('\n');
fprintf('============================================================\n');
fprintf(' Professional V2 - Project Initialization\n');
fprintf('============================================================\n');

%% 1. Determine project locations

% This file is located at:
% Professional V2\MATLAB\InitializeProject.m
%
% Therefore:
% matlabRoot  = Professional V2\MATLAB
% projectRoot = Professional V2

matlabRoot = fileparts(mfilename('fullpath'));
projectRoot = fileparts(matlabRoot);

% Simulink directory
simulinkRoot = fullfile(projectRoot, 'Simulink');

% Phase 02 PI controller model
modelPath = fullfile( ...
    simulinkRoot, ...
    'CruiseControl_PI_Controller.slx');


%% 2. Validate project root

if ~isfolder(projectRoot)
    error( ...
        'InitializeProject:InvalidProjectRoot', ...
        'Project root does not exist:\n%s', ...
        projectRoot);
end


%% 3. Validate MATLAB directory

if ~isfolder(matlabRoot)
    error( ...
        'InitializeProject:MissingMATLABDirectory', ...
        'MATLAB directory does not exist:\n%s', ...
        matlabRoot);
end


%% 4. Validate Simulink directory

if ~isfolder(simulinkRoot)
    error( ...
        'InitializeProject:MissingSimulinkDirectory', ...
        'Simulink directory does not exist:\n%s', ...
        simulinkRoot);
end


%% 5. Add MATLAB project folders to MATLAB path

addpath(genpath(matlabRoot));
rehash;


%% 6. Store project paths in base workspace

assignin('base', 'projectRoot', projectRoot);
assignin('base', 'matlabRoot', matlabRoot);
assignin('base', 'simulinkRoot', simulinkRoot);
assignin('base', 'modelPath', modelPath);


%% 7. Display resolved project paths

fprintf('\n[PROJECT PATHS]\n');
fprintf('Project Root:\n%s\n\n', projectRoot);
fprintf('MATLAB Root:\n%s\n\n', matlabRoot);
fprintf('Simulink Root:\n%s\n\n', simulinkRoot);
fprintf('PI Model Path:\n%s\n', modelPath);


%% 8. Verify required MATLAB functions

fprintf('\n[MATLAB FUNCTION VERIFICATION]\n');

requiredFunctions = { ...
    'Parameters', ...
    'BuildPlant', ...
    'DesignPController', ...
    'DesignPIController', ...
    'ValidateSimulink', ...
    'ValidatePIController'};

allFunctionsFound = true;

for k = 1:numel(requiredFunctions)

    functionName = requiredFunctions{k};
    functionPath = which(functionName);

    if isempty(functionPath)

        fprintf('  %s : NOT FOUND\n', functionName);
        allFunctionsFound = false;

    else

        fprintf('  %s : FOUND\n', functionName);
        fprintf('    %s\n', functionPath);

    end

end


%% 9. Verify Phase 02 Simulink model

fprintf('\n[SIMULINK MODEL VERIFICATION]\n');

if exist(modelPath, 'file') == 2

    fprintf('Model: CruiseControl_PI_Controller.slx\n');
    fprintf('Path : %s\n', modelPath);
    fprintf('Status: FOUND\n');

else

    fprintf('Model: CruiseControl_PI_Controller.slx\n');
    fprintf('Path : %s\n', modelPath);
    fprintf('Status: NOT FOUND\n');

    error( ...
        'InitializeProject:MissingPIModel', ...
        ['Phase 02 Simulink model was not found.' ...
         '\nExpected location:\n%s'], ...
        modelPath);

end


%% 10. Final verification

if ~allFunctionsFound

    error( ...
        'InitializeProject:MissingMATLABFunctions', ...
        ['One or more required MATLAB functions were not found ' ...
         'on the MATLAB path.']);

end


fprintf('\n============================================================\n');
fprintf(' Project Initialization: PASS\n');
fprintf('============================================================\n\n');

end

