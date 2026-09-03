function sys = BuildLQRPlant(params)

%==========================================================================
% BuildLQRPlant
%
% Purpose:
%   Build the actuator-aware state-space model used for LQR design.
%
% States:
%   x(1) = vehicle speed [m/s]
%   x(2) = actuator force [N]
%
% Model:
%
%   m*dv/dt + b*v = F
%
%   tau_a*dF/dt + F = u
%
% Therefore:
%
%   dx/dt = A*x + B*u
%   y     = C*x + D*u
%
%==========================================================================

%% Extract physical parameters

m     = params.mass;
b     = params.drag;
tau_a = params.actuatorTimeConstant;

%% State-space model

A = [ -b/m       1/m;
    0        -1/tau_a ];

B = [ 0;
    1/tau_a ];

C = [ 1  0 ];

D = 0;

%% Create state-space system

sys = ss(A,B,C,D);

%% Store engineering information

sys.InputName  = {'ActuatorCommand'};
sys.OutputName = {'VehicleSpeed'};

sys.StateName = {
    'VehicleSpeed'
    'ActuatorForce'
    };

%% Display model

fprintf('\n');
fprintf('============================================================\n');
fprintf('CRUISE CONTROL LQR STATE-SPACE MODEL\n');
fprintf('============================================================\n');

fprintf('Mass                 m     = %.4f kg\n',m);
fprintf('Damping              b     = %.4f Ns/m\n',b);
fprintf('Actuator time const. tau_a = %.4f s\n',tau_a);

fprintf('\nA = \n');
disp(A);

fprintf('B = \n');
disp(B);

fprintf('C = \n');
disp(C);

fprintf('D = \n');
disp(D);

fprintf('============================================================\n\n');

end