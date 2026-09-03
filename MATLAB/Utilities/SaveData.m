function SaveData(data, fileName, saveFolder)
%==========================================================================
% SAVEDATA
%==========================================================================
% Project : Professional V2 - Automotive Cruise Control
% File    : SaveData.m
% Purpose : Save simulation and validation data.
%
% Inputs:
%   data      : Structure containing simulation results
%   fileName  : MAT filename
%   saveFolder: Destination folder
%
%==========================================================================


%% Input Validation

assert(isstruct(data), ...
    'SaveData:InvalidInput', ...
    'Input data must be a structure.');

validateattributes(fileName,...
    {'char','string'},...
    {'nonempty'});


%% Create Folder

if ~exist(saveFolder,'dir')
    mkdir(saveFolder);
end


%% Save Data

filePath = fullfile(saveFolder,fileName);

save(filePath,'-struct','data');


%% Confirmation

fprintf('Data saved:\n%s\n',filePath);


end