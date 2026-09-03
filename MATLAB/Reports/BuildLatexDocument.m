function BuildLatexDocument( ...
    latexFile,...
    projectRoot,...
    controller,...
    validation,...
    sensitivityResults,...
    figureFolder)

%==========================================================================
% Phase 01 Professional Engineering Design Review Report
% Automotive Cruise Control - P Controller
%
%==========================================================================

fprintf('\nGenerating Professional Phase 01 Report...\n');


%% Load Control Effort Data

dataFile = fullfile( ...
    projectRoot,...
    'Results',...
    'Data',...
    'ControlEffort_Data.mat');


if isfile(dataFile)

    load(dataFile,"results");

    controlResults = results;

else

    controlResults = [];

end



%% Basic Data

Kp = controller.Kp;


maxError = validation.MaxError;
rmse     = validation.RMSE;


if validation.Pass

    validationStatus = "PASS";

else

    validationStatus = "FAIL";

end



%% Selected Controller Performance

selectedIndex = find([sensitivityResults.Kp] == Kp);


if isempty(selectedIndex)

    selectedIndex = 1;

end


performance = sensitivityResults(selectedIndex);



%% Control Effort

if ~isempty(controlResults)

    effortIndex = find([controlResults.Kp]==Kp);

    if isempty(effortIndex)
        effortIndex = 1;
    end

    effort = controlResults(effortIndex);

else

    effort.MaximumEffort = NaN;
    effort.RMSEffort = NaN;

end



%% Open Latex File


fid = fopen(latexFile,'w');


assert(fid~=-1,...
    "Cannot create LaTeX file");



%% Latex Header


fprintf(fid,'\\documentclass[11pt]{article}\n');

fprintf(fid,'\\usepackage{graphicx}\n');

fprintf(fid,'\\usepackage{booktabs}\n');

fprintf(fid,'\\usepackage{amsmath}\n');

fprintf(fid,'\\usepackage{geometry}\n');
fprintf(fid,'\\geometry{margin=1in}\n');

% Image path
fprintf(fid,'\\graphicspath{{%s/}}\n',...
    strrep(figureFolder,'\','/'));

fprintf(fid,'\\begin{document}\n\n');



%% Title


fprintf(fid,'\\begin{center}\n');

fprintf(fid,'{\\Large\\textbf{Phase 01 Engineering Design Review}}\\\\[5mm]\n');

fprintf(fid,'{\\large Automotive Cruise Control System - P Controller Development}\n');

fprintf(fid,'\\end{center}\n\n');



%% Executive Summary


fprintf(fid,'\\section{Executive Summary}\n');


fprintf(fid,...
['This report documents the design, implementation and validation ',...
'of a proportional controller for an automotive cruise control system. ']);

fprintf(fid,...
['The complete workflow includes mathematical modelling, MATLAB ',...
'implementation, Simulink validation, sensitivity analysis and engineering decision making.']);


fprintf(fid,'\n\n');



%% System Description


fprintf(fid,'\\section{Vehicle System Model}\n');


fprintf(fid,...
['The vehicle longitudinal dynamics are represented by a first order ',...
'linear model:']);


fprintf(fid,'\n\n');


fprintf(fid,'\\[');

fprintf(fid,'m\\frac{dv}{dt}+bv=u');

fprintf(fid,'\\]\n\n');


fprintf(fid,...
'The transfer function is:');


fprintf(fid,'\n\n');


fprintf(fid,'\\[');

fprintf(fid,'G(s)=\\frac{1}{ms+b}');

fprintf(fid,'\\]\n\n');



%% Controller


fprintf(fid,'\\section{Controller Design}\n');


fprintf(fid,...
'A proportional controller was selected for the first design phase:');


fprintf(fid,'\n\n');


fprintf(fid,'\\[');

fprintf(fid,'C(s)=K_p');

fprintf(fid,'\\]\n\n');


fprintf(fid,...
'Selected gain:');


fprintf(fid,'\n\n');


fprintf(fid,'\\[');

fprintf(fid,'K_p = %.0f',Kp);

fprintf(fid,'\\]');


fprintf(fid,'\n\n');

%% ========================================================================
% Closed Loop Performance
% ========================================================================


fprintf(fid,'\\section{Closed Loop Performance}\n');


fprintf(fid,...
'The selected controller performance was evaluated using automated metrics.\n\n');



fprintf(fid,'\\begin{table}[h]\n');
fprintf(fid,'\\centering\n');


fprintf(fid,'\\begin{tabular}{lc}\n');

fprintf(fid,'\\toprule\n');

fprintf(fid,'Parameter & Value\\\\\n');

fprintf(fid,'\\midrule\n');


fprintf(fid,...
'Rise Time & %.3f s\\\\\n',...
performance.RiseTime);


fprintf(fid,...
'Settling Time & %.3f s\\\\\n',...
performance.SettlingTime);


fprintf(fid,...
'Overshoot & %.2f \\%%\\\\\n',...
performance.Overshoot);


fprintf(fid,...
'Steady State Error & %.4f\\\\\n',...
performance.SteadyStateError);


fprintf(fid,...
'Closed Loop Pole & %.3f\\\\\n',...
performance.Pole);


fprintf(fid,'\\bottomrule\n');

fprintf(fid,'\\end{tabular}\n');

fprintf(fid,'\\end{table}\n\n');





%% ========================================================================
% MATLAB Simulink Validation
% ========================================================================


fprintf(fid,'\\section{MATLAB and Simulink Validation}\n');


fprintf(fid,...
['The MATLAB analytical model was compared with the Simulink ',...
'implementation to verify modelling consistency.']);

fprintf(fid,'\n\n');


fprintf(fid,'\\begin{table}[h]\n');

fprintf(fid,'\\centering\n');

fprintf(fid,'\\begin{tabular}{lc}\n');

fprintf(fid,'\\toprule\n');

fprintf(fid,'Metric & Value\\\\\n');

fprintf(fid,'\\midrule\n');


fprintf(fid,...
'Maximum Error & %.4e\\\\\n',...
maxError);


fprintf(fid,...
'RMSE & %.4e\\\\\n',...
rmse);


fprintf(fid,...
'Validation Result & %s\\\\\n',...
validationStatus);


fprintf(fid,'\\bottomrule\n');

fprintf(fid,'\\end{tabular}\n');

fprintf(fid,'\\end{table}\n\n');



addFigure(fid,...
    'MATLAB_vs_Simulink.png',...
    'MATLAB and Simulink response comparison');


addFigure(fid,...
    'Validation_Error.png',...
    'Validation error between MATLAB and Simulink');





%% ========================================================================
% Kp Sensitivity Analysis
% ========================================================================


fprintf(fid,'\\section{Kp Sensitivity Analysis}\n');


fprintf(fid,...
['A proportional gain sweep was performed to understand the ',...
'influence of controller gain on transient response, accuracy and stability.']);

fprintf(fid,'\n\n');



fprintf(fid,'\\begin{table}[h]\n');

fprintf(fid,'\\centering\n');


fprintf(fid,'\\begin{tabular}{cccccc}\n');

fprintf(fid,'\\toprule\n');

fprintf(fid,...
'$K_p$ & Rise(s) & Settle(s) & Overshoot & Error & Pole\\\\\n');


fprintf(fid,'\\midrule\n');



for i=1:length(sensitivityResults)

    r=sensitivityResults(i);

    fprintf(fid,...
    '%.0f & %.2f & %.2f & %.2f & %.4f & %.3f\\\\\n',...
    r.Kp,...
    r.RiseTime,...
    r.SettlingTime,...
    r.Overshoot,...
    r.SteadyStateError,...
    r.Pole);

end



fprintf(fid,'\\bottomrule\n');

fprintf(fid,'\\end{tabular}\n');

fprintf(fid,'\\end{table}\n\n');



addFigure(fid,...
    'Performance_Summary.png',...
    'Performance variation with proportional gain');


addFigure(fid,...
    'Pole_Map.png',...
    'Closed loop pole movement with increasing Kp');



%% ========================================================================
% Control Effort
% ========================================================================


fprintf(fid,'\\section{Control Effort Analysis}\n');


fprintf(fid,...
['Controller effort was analysed because increasing Kp improves ',...
'response speed but increases actuator demand.']);

fprintf(fid,'\n\n');



fprintf(fid,'\\begin{table}[h]\n');

fprintf(fid,'\\centering\n');

fprintf(fid,'\\begin{tabular}{lc}\n');

fprintf(fid,'\\toprule\n');

fprintf(fid,'Parameter & Value\\\\\n');

fprintf(fid,'\\midrule\n');


fprintf(fid,...
'Maximum Control Effort & %.3f\\\\\n',...
effort.MaximumEffort);


fprintf(fid,...
'RMS Control Effort & %.3f\\\\\n',...
effort.RMSEffort);



fprintf(fid,'\\bottomrule\n');

fprintf(fid,'\\end{tabular}\n');

fprintf(fid,'\\end{table}\n\n');



addFigure(fid,...
    'Control_Effort.png',...
    'Controller output signal');


addFigure(fid,...
    'ControlEffort_vs_Kp.png',...
    'Control effort variation with Kp');




%% ========================================================================
% Engineering Decision
% ========================================================================


fprintf(fid,'\\section{Engineering Decision}\n');


fprintf(fid,...
['The selected gain Kp = %.0f provides the best compromise ',...
'between response speed, steady-state accuracy and actuator demand.'],...
Kp);


fprintf(fid,'\n\n');


fprintf(fid,...
['Increasing Kp beyond this value improves transient response; ',...
'however, it increases control effort and reduces robustness margin.']);


fprintf(fid,'\n\n');


fprintf(fid,...
['The proportional controller remains limited because it cannot ',...
'eliminate steady-state tracking error.']);


fprintf(fid,'\n\n');


fprintf(fid,...
['Therefore, the next development phase introduces integral ',...
'action through PI controller design.']);





%% ========================================================================
% Conclusion
% ========================================================================


fprintf(fid,'\\section{Conclusion}\n');


fprintf(fid,...
['Phase 01 successfully demonstrated the complete control ',...
'engineering workflow from modelling to validation.']);


fprintf(fid,'\n\n');


fprintf(fid,...
['The P controller provides stable closed-loop behaviour, ',...
'but PI control is required to achieve zero steady-state error.']);





%% Close


fprintf(fid,'\n\\end{document}\n');


fclose(fid);


fprintf('\nProfessional LaTeX report generated:\n%s\n',latexFile);



end



%% ========================================================================
% Local Figure Function
% ========================================================================


function addFigure(fid,fileName,caption)


fprintf(fid,'\\begin{figure}[h]\n');

fprintf(fid,'\\centering\n');


fprintf(fid,...
'\\includegraphics[width=0.75\\linewidth]{%s}\n',...
fileName);



fprintf(fid,...
'\\caption{%s}\n',...
caption);


fprintf(fid,'\\end{figure}\n\n');


end