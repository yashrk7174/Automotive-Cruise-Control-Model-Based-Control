function GeneratePDFReport()
%==========================================================================
% GeneratePDFReport
%
% Project:
% Automotive Cruise Control - Professional V2
%
% Purpose:
% Generate Phase01 engineering report LaTeX source file.
%
% Output:
% Results/Reports/Phase01_Report.tex
%
%==========================================================================


clc;


%% ================= PROJECT ROOT =================

matlabFolder = fileparts(mfilename('fullpath'));

projectRoot = fileparts(matlabFolder);



%% ================= PATH DEFINITIONS =================

resultsFolder = fullfile(projectRoot,"Results");

figureFolder = fullfile(resultsFolder,"Figures");

dataFolder = fullfile(resultsFolder,"Data");

reportFolder = fullfile(resultsFolder,"Reports");



%% ================= INPUT FILES =================

metadataFile = fullfile( ...
    reportFolder,...
    "Phase01_Report_Metadata.mat");


% Compatibility check for previous name
if ~isfile(metadataFile)

    oldMetadataFile = fullfile( ...
        reportFolder,...
        "Phase01_Report_Data.mat");

    if isfile(oldMetadataFile)

        metadataFile = oldMetadataFile;

    end

end


controllerDataFile = fullfile( ...
    dataFolder,...
    "Controller_Data.mat");



assert(isfile(metadataFile),...
    "No Phase01 report metadata file found in Results/Reports");


assert(isfile(controllerDataFile),...
    "Missing Controller_Data.mat");

%% ================= FILE VALIDATION =================

assert(isfile(metadataFile),...
    "Missing Phase01_Report_Metadata.mat");


assert(isfile(controllerDataFile),...
    "Missing Controller_Data.mat");



%% ================= LOAD DATA =================

load(metadataFile,"report");

load(controllerDataFile);



%% ================= CREATE TEX FILE =================

texFile = fullfile( ...
    reportFolder,...
    "Phase01_Report.tex");


fid = fopen(texFile,'w');


assert(fid ~= -1,...
    "Unable to create LaTeX file");



%% ================= DOCUMENT HEADER =================

fprintf(fid,'\\documentclass[11pt]{article}\n');

fprintf(fid,'\\usepackage{graphicx}\n');

fprintf(fid,'\\usepackage{booktabs}\n');

fprintf(fid,'\\usepackage{amsmath}\n');

fprintf(fid,'\\usepackage[a4paper,margin=1in]{geometry}\n');

fprintf(fid,'\\begin{document}\n');



fprintf(fid,'\\title{Automotive Cruise Control System\\\\Professional Version 2}\n');

fprintf(fid,'\\author{Control Engineering Project}\n');

fprintf(fid,'\\date{\\today}\n');

fprintf(fid,'\\maketitle\n');



%% ================= PROJECT OBJECTIVE =================

fprintf(fid,'\\section{Project Objective}\n');

fprintf(fid,...
'Development and validation of a MATLAB/Simulink based automotive cruise control system using a proportional controller.\n');



%% ================= MATHEMATICAL MODEL =================

fprintf(fid,'\\section{Mathematical Model}\n');

fprintf(fid,...
'The vehicle longitudinal dynamics are represented as:\n');


fprintf(fid,...
'\\begin{equation}\n');


fprintf(fid,...
'G(s)=\\frac{1}{ms+b}\n');


fprintf(fid,...
'\\end{equation}\n');



%% ================= CONTROLLER =================

fprintf(fid,'\\section{Controller Design}\n');

fprintf(fid,...
'The implemented controller is a proportional controller:\n');


fprintf(fid,...
'\\begin{equation}\n');

fprintf(fid,...
'C(s)=K_p\n');

fprintf(fid,...
'\\end{equation}\n');


fprintf(fid,...
'Controller Gain: $K_p = %.2f$\n',...
report.Kp);



%% ================= PERFORMANCE METRICS =================

fprintf(fid,'\\section{Closed Loop Performance}\n');


fprintf(fid,'\\begin{tabular}{ll}\n');

fprintf(fid,'\\toprule\n');

fprintf(fid,'Parameter & Value\\\\\n');

fprintf(fid,'\\midrule\n');


fprintf(fid,...
'Rise Time & %.4f s\\\\\n',...
report.Performance.RiseTime);


fprintf(fid,...
'Settling Time & %.4f s\\\\\n',...
report.Performance.SettlingTime);


fprintf(fid,...
'Overshoot & %.2f \\%%\\\\\n',...
report.Performance.Overshoot);



fprintf(fid,'\\bottomrule\n');

fprintf(fid,'\\end{tabular}\n');



%% ================= VALIDATION =================

fprintf(fid,'\\section{MATLAB vs Simulink Validation}\n');


fprintf(fid,...
'Maximum Error = %.3e\\\\\n',...
report.Validation.MaxError);


fprintf(fid,...
'RMSE = %.3e\\\\\n',...
report.Validation.RMSE);



if report.Validation.Pass

    fprintf(fid,...
    'Validation Result: PASS\\\\\n');

else

    fprintf(fid,...
    'Validation Result: FAIL\\\\\n');

end



%% ================= FIGURES =================

fprintf(fid,'\\section{Simulation Results}\n');


figures = [

"ClosedLoop_Response.png"
"MATLAB_vs_Simulink.png"
"Validation_Error.png"

];


for i = 1:length(figures)


    figPath = fullfile(figureFolder,figures(i));


    if isfile(figPath)

        fprintf(fid,...
        '\\begin{figure}[h]\n');


        fprintf(fid,...
        '\\centering\n');


        fprintf(fid,...
        '\\includegraphics[width=0.85\\linewidth]{../Figures/%s}\n',...
        figures(i));


        fprintf(fid,...
        '\\caption{%s}\n',...
        erase(figures(i),".png"));


        fprintf(fid,...
        '\\end{figure}\n');

    end

end



%% ================= CONCLUSION =================

fprintf(fid,'\\section{Conclusion}\n');


fprintf(fid,...
'The proportional controller provides stable cruise control operation. ');


fprintf(fid,...
'The remaining steady-state error motivates future PI controller development.\n');



%% ================= CLOSE FILE =================

fprintf(fid,'\\end{document}\n');


fclose(fid);



fprintf("\n");
fprintf("====================================\n");
fprintf(" Phase01 LaTeX Report Generated\n");
fprintf("====================================\n");

fprintf("%s\n",texFile);

fprintf("====================================\n");


end