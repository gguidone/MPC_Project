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
    eta_A = params.exercise.etaA;
    T0_min = params.exercise.To_min;
    T0_max = params.exercise.To_max;
    T0 = params.exercise.To;
    alpha = [params.model.a1o;params.model.a2o;params.model.a3o];
    Bd = params.model.Bd;
    d_min = alpha * T0_min + eta_min;
    d_max = alpha * T0_max + eta_max;
    d =( alpha * T0 + eta_A);
    params_robust.constraints.DisturbanceMatrix = [inv(Bd)*eye(params.model.nx);-inv(Bd)*eye(params.model.nx)];
    params_robust.constraints.DisturbanceRHS = [(d_max - d);-(d_min - d)];
    
end