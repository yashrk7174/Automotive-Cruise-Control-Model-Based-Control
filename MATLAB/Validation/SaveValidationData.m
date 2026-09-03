function SaveValidationData(validation)

%==========================================================================
% SaveValidationData
%
% Purpose:
%   Save MATLAB vs Simulink validation results.
%
% Input:
%   validation structure from ValidateSimulink
%
% Output:
%   Results/Data/Validation_Data.mat
%
%==========================================================================


%% Input Check

assert(isstruct(validation),...
    'Validation input must be a structure.');



%% Project Path

projectRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));


dataFolder = fullfile(...
    projectRoot,...
    'Results',...
    'Data');


if ~exist(dataFolder,'dir')
    mkdir(dataFolder);
end



%% Save

saveFile = fullfile(...
    dataFolder,...
    'Validation_Data.mat');


save(saveFile,'validation');


fprintf('\n');
fprintf('Validation data saved:\n');
fprintf('%s\n',saveFile);
fprintf('\n');


end