function CompileLatexPDF(latexFile,reportFolder)

%==========================================================================
% CompileLatexPDF
%
% Compile LaTeX report using MiKTeX
%
%==========================================================================


fprintf('\nCompiling LaTeX PDF...\n');


%% Check

assert(isfile(latexFile),...
    'LaTeX file missing');



%% Save current directory

oldFolder = pwd;


%% Move to report directory

cd(reportFolder);



%% Extract filename only

[~,name,ext] = fileparts(latexFile);


texName = [name ext];



%% Compile using local filename

command = sprintf( ...
    'pdflatex -interaction=nonstopmode "%s"',...
    texName);



[status,result] = system(command);



%% Return

cd(oldFolder);



%% Check result

if status ~= 0 && ~isfile(strrep(latexFile,'.tex','.pdf'))

    fprintf('\n');
    fprintf('LATEX COMPILATION FAILED\n');
    fprintf('%s\n',result);

    error('PDF generation failed');

end



fprintf('\nPDF generated successfully:\n');

fprintf('%s.pdf\n',name);



end