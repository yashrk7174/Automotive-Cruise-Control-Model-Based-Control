function controller = DesignFilteredPIDController(Kp,Ki,Kd,N)

%% ================================================================
% DesignFilteredPIDController
%
% Creates a practical PID controller with filtered derivative action.
%
% Controller:
%
%       Ki        Kd*N*s
% C(s) = -- + Kp + -------
%        s         s + N
%
% Inputs:
%   Kp - proportional gain
%   Ki - integral gain
%   Kd - derivative gain
%   N  - derivative filter coefficient
%
% Output:
%   controller.C
%
% ================================================================


%% Validate inputs

if nargin ~= 4
    error("Expected Kp, Ki, Kd and N.");
end

if Kp < 0 || Ki < 0 || Kd < 0
    error("PID gains must be non-negative.");
end

if N <= 0
    error("Derivative filter coefficient N must be positive.");
end


%% Create Laplace variable

s = tf('s');


%% Filtered PID controller

C = Kp + Ki/s + Kd*(N*s)/(s + N);


%% Store controller information

controller.C = minreal(C);

controller.Kp = Kp;
controller.Ki = Ki;
controller.Kd = Kd;
controller.N = N;


%% Display controller

fprintf("\n");
fprintf("=============================================\n");
fprintf("FILTERED PID CONTROLLER\n");
fprintf("=============================================\n");

fprintf("Kp = %.4f\n",Kp);
fprintf("Ki = %.4f\n",Ki);
fprintf("Kd = %.4f\n",Kd);
fprintf("N  = %.4f\n",N);

fprintf("=============================================\n\n");

end