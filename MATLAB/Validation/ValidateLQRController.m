function validation = ValidateLQRController()

%==========================================================================
% ValidateLQRController
%
% Final validation of actuator-aware LQR controller.
%
% Workflow:
%   Parameters -> LQR Plant -> LQR Controller -> MATLAB simulation
%
% No modification of existing project models is required.
%==========================================================================

clc;

fprintf('\n============================================================\n');
fprintf('LQR CONTROLLER VALIDATION\n');
fprintf('============================================================\n');

%% 1. Initialize project

InitializeProject;

params = Parameters;

%% 2. Build LQR plant

sys = BuildLQRPlant(params);

A = sys.A;
B = sys.B;
C = sys.C;
D = sys.D;

%% 3. LQR design

Q = [1 0;
     0 0];

R = 2.0408e-6;

controller = DesignLQRController(A,B,Q,R);

K = controller.K;

%% 4. Reference tracking gain

Acl = A - B*K;

Nbar = -1/(C*(Acl\B));

fprintf('\nLQR Results:\n');
fprintf('K = [%f  %f]\n',K(1),K(2));
fprintf('Nbar = %f N\n',Nbar);

%% 5. MATLAB simulation with actuator saturation

t = (0:0.001:120)';

x = zeros(2,length(t));

r = 1;

for k = 1:length(t)-1

    u_raw = -K*x(:,k) + Nbar*r;

    u = min(max(u_raw,...
        -params.maxActuatorForce),...
         params.maxActuatorForce);

    dx = A*x(:,k) + B*u;

    x(:,k+1) = x(:,k) + dx*(t(k+1)-t(k));

end

y = C*x;

u_raw = zeros(size(t));

u_sat = zeros(size(t));

for k = 1:length(t)

    u_raw(k) = -K*x(:,k) + Nbar*r;

    u_sat(k) = min(max(u_raw(k),...
        -params.maxActuatorForce),...
         params.maxActuatorForce);

end

%% 6. Performance metrics

info = stepinfo(y,t,r);

finalSpeed = y(end);

SSE = abs(r-finalSpeed);

controlPeak = max(abs(u_sat));

saturationActive = any(abs(u_raw) > params.maxActuatorForce);

saturationSamples = sum(abs(u_raw) > params.maxActuatorForce);

saturationPercentage = ...
    100*saturationSamples/length(t);

%% 7. Display results

fprintf('\n------------------------------------------------------------\n');
fprintf('PERFORMANCE RESULTS\n');
fprintf('------------------------------------------------------------\n');

fprintf('Rise Time             : %.6f s\n',info.RiseTime);
fprintf('Settling Time         : %.6f s\n',info.SettlingTime);
fprintf('Overshoot             : %.6f %%\n',info.Overshoot);
fprintf('Final Speed           : %.9f m/s\n',finalSpeed);
fprintf('Steady-State Error    : %.9f m/s\n',SSE);

fprintf('\nCONTROL EFFORT\n');
fprintf('Raw Control Peak      : %.6f N\n',max(abs(u_raw)));
fprintf('Actual Control Peak   : %.6f N\n',controlPeak);
fprintf('Actuator Limit        : %.6f N\n',...
    params.maxActuatorForce);

fprintf('Saturation Active     : %s\n',...
    string(saturationActive));

fprintf('Saturated Samples     : %d / %d\n',...
    saturationSamples,length(t));

fprintf('Saturation Percentage : %.4f %%\n',...
    saturationPercentage);

%% 8. Validation criteria

trackingPass = SSE < 1e-3;

stabilityPass = all(real(controller.closedLoopPoles) < 0);

actuatorLimitPass = controlPeak <= ...
    params.maxActuatorForce + 1e-9;

fprintf('\n------------------------------------------------------------\n');
fprintf('VALIDATION STATUS\n');
fprintf('------------------------------------------------------------\n');

fprintf('Closed-loop Stability : %s\n',...
    string(stabilityPass));

fprintf('Tracking              : %s\n',...
    string(trackingPass));

fprintf('Actuator Output       : %s\n',...
    string(actuatorLimitPass));

%% 9. Engineering interpretation

if saturationActive

    fprintf('\nNOTE:\n');
    fprintf(['Ideal LQR command exceeds the actuator limit.\n' ...
             'The actuator saturation is therefore active.\n']);

    fprintf('Maximum raw demand = %.6f N\n',...
        max(abs(u_raw)));

    fprintf('Actuator limit      = %.6f N\n',...
        params.maxActuatorForce);

end

%% 10. Final validation decision

validation = struct();

validation.K = K;
validation.Nbar = Nbar;
validation.finalSpeed = finalSpeed;
validation.SSE = SSE;

validation.riseTime = info.RiseTime;
validation.settlingTime = info.SettlingTime;
validation.overshoot = info.Overshoot;

validation.rawControlPeak = max(abs(u_raw));
validation.actualControlPeak = controlPeak;

validation.saturationActive = saturationActive;
validation.saturationPercentage = saturationPercentage;

validation.stabilityPass = stabilityPass;
validation.trackingPass = trackingPass;
validation.actuatorLimitPass = actuatorLimitPass;

validation.PASS = ...
    stabilityPass && ...
    trackingPass && ...
    actuatorLimitPass;

fprintf('\n============================================================\n');

if validation.PASS
    fprintf('LQR VALIDATION: PASS\n');
else
    fprintf('LQR VALIDATION: CHECK REQUIRED\n');
end

fprintf('============================================================\n\n');

end