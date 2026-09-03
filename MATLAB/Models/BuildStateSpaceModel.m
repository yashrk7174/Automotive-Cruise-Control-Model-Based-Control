function sys = BuildStateSpaceModel()

%% ================================================================
% BuildStateSpaceModel
%
% Creates the state-space representation of the cruise-control plant.
%
% Vehicle model:
%
%       m*dv/dt + b*v = u
%
% Rearranged:
%
%       dv/dt = -(b/m)*v + (1/m)*u
%
% State:
%
%       x = v
%
% State-space form:
%
%       x_dot = A*x + B*u
%       y     = C*x + D*u
%
% ================================================================

%% Load project parameters

params = Parameters();

m = params.mass;
b = params.drag;

%% State-space matrices

A = -b/m;
B =  1/m;
C =  1;
D =  0;

%% Create state-space model

sys = ss(A,B,C,D);

%% Display model

fprintf('\n');
fprintf('============================================================\n');
fprintf('CRUISE CONTROL STATE-SPACE MODEL\n');
fprintf('============================================================\n');

fprintf('Mass        m = %.4f kg\n',m);
fprintf('Damping     b = %.4f Ns/m\n',b);

fprintf('\nState-space matrices:\n\n');

disp('A = ');
disp(A);

disp('B = ');
disp(B);

disp('C = ');
disp(C);

disp('D = ');
disp(D);

fprintf('============================================================\n\n');

end
