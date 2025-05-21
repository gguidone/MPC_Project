%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2025, Amon Lahr, Simon Muntwiler, Antoine Leeman, Fabian Flürenbrock & Marco Heim Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [Xsim,Usim,ctrl_info] = simulate_uncertain(x0, ctrl, Wsim, params_robust)
	% YOUR CODE HERE
    params = params_robust;
    N_sim = params.exercise.SimHorizon;
    X = zeros(params.model.nx, N_sim + 1)';
    U = zeros(params.model.nu, N_sim)';
    default = struct('ctrl_feas', true);
    ctrl_info = repmat(default, N_sim, 1);
    X(1,:) = x0;
    x_i = x0;
    for i_time = 1: N_sim
        u_i = ctrl.eval(x_i);
        x_i = params.model.A * x_i + params.model.B * u_i + Wsim(i_time);
        X(i_time+1,:) = x_i;
        U(i_time, :) = u_i;
    end
    Xsim = X';
    Usim = U';
end