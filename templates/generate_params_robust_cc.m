%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2025, Amon Lahr, Simon Muntwiler, Antoine Leeman, Fabian Flürenbrock & Marco Heim Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [params_robust] = generate_params_robust_cc(params)
    %------- DON'T CHANGE------------------------
    params_robust = params;
    %--------------------------------------------
    % Implement your solution here:
    eta_min = params.exercise.eta_min;
    eta_max = params.exercise.eta_max;
    T0_min = params.exercise.To_min;
    T0_max = params.exercise.To_max;
    alpha = [params.model.a1o;params.model.a2o;params.model.a3o];
    Bd = params.model.Bd;
    params_robust.constraints.DisturbanceMatrix = [eye(params.model.nx);-eye(params.model.nx)];
    params_robust.constraints.DisturbanceRHS = [Bd*(alpha*T0_max+eta_max);-Bd*(alpha*T0_min+eta_min)];
end