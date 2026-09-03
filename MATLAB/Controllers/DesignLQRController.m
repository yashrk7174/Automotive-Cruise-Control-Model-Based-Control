function controller = DesignLQRController(A,B,Q,R)

%% ================================================================
% DesignLQRController
%
% LQR controller design for automotive cruise control.
%
% Control law:
%
%       u = -Kx
%
% Cost function:
%
%       J = integral(x'Qx + u'Ru) dt
%
% ================================================================

if nargin ~= 4
    error("Expected A, B, Q and R.");
end

% Validate dimensions
n = size(A,1);

assert(size(A,2) == n, ...
    'A must be square.');

assert(size(B,1) == n, ...
    'A and B dimensions are inconsistent.');

assert(all(eig(Q) >= -1e-10), ...
    'Q must be positive semi-definite.');

assert(R > 0, ...
    'R must be positive.');

% Controllability check
Co = ctrb(A,B);
controllabilityRank = rank(Co);

if controllabilityRank < n
    error("LQR design requires a controllable system.");
end

% LQR design
[K,S,e] = lqr(A,B,Q,R);

% Store results
controller.K = K;
controller.Q = Q;
controller.R = R;
controller.S = S;
controller.closedLoopPoles = e;
controller.controllabilityRank = controllabilityRank;

%% Display

fprintf("\n");
fprintf("============================================================\n");
fprintf("LQR CONTROLLER DESIGN\n");
fprintf("============================================================\n");

fprintf("Q = \n");
disp(Q);

fprintf("R = \n");
disp(R);

fprintf("K = \n");
disp(K);

fprintf("Closed-loop poles = \n");
disp(e);

fprintf("Controllability rank = %d / %d\n", ...
    controllabilityRank,n);

fprintf("============================================================\n\n");

end