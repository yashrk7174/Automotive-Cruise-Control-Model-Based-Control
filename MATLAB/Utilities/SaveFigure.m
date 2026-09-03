function SaveFigure(figHandle, fileName, folderPath)
%==========================================================================
% SaveFigure
%
% Purpose:
%   Standardized figure saving utility.
%
% Inputs:
%   figHandle  - Figure handle
%   fileName   - Figure name without extension
%   folderPath - Destination folder
%
% Output:
%   PNG figure file
%
%==========================================================================


%% Input validation

assert(isgraphics(figHandle,'figure'),...
    'Invalid figure handle.');

assert(ischar(fileName) || isstring(fileName),...
    'File name must be text.');

assert(isfolder(folderPath),...
    'Figure folder does not exist.');



%% Add extension automatically

if ~endsWith(fileName,'.png')

    fileName = fileName + ".png";

end



%% Build complete path

fullPath = fullfile(folderPath,fileName);



%% Save Figure

exportgraphics( ...
    figHandle,...
    fullPath,...
    'Resolution',300);



fprintf("Saved Figure: %s\n",fullPath);


end