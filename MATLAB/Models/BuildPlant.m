function plant = BuildPlant(params)
%==========================================================================
% BUILDPLANT
%==========================================================================
% Project : Professional V2 - Automotive Cruise Control
% File    : BuildPlant.m
% Purpose : Construct the vehicle transfer function from physical parameters.
%
% Author  : Yash Khiste
% Version : Professional V2
%
% Inputs:
%   params : Structure returned by Parameters.m
%
% Outputs:
%   plant  : Continuous-time transfer function of the vehicle
%
% Dependencies:
%   Parameters.m
%   Control System Toolbox
%
%==========================================================================

%% Input Validation

validateattributes(params, {'struct'}, {'nonempty'}, mfilename, 'params');

requiredFields = {'mass', 'drag'};

for k = 1:numel(requiredFields)
    assert(isfield(params, requiredFields{k}), ...
        'BuildPlant:MissingField', ...
        'Missing required field "%s" in parameter structure.', ...
        requiredFields{k});
end

%% Plant Construction
%
% Vehicle Dynamics:
%
%        1
% G(s) = ------
%        m*s+b
%

plant = tf(1, [params.mass, params.drag]);

%% Model Validation

assert(order(plant) == 1, ...
    'BuildPlant:InvalidModel', ...
    'Unexpected transfer function order.');

end