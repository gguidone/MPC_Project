%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2025, Amon Lahr, Simon Muntwiler, Antoine Leeman, Fabian Flürenbrock & Marco Heim Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [x_s, u_s] = compute_steady_state(params, d)

A_steady_state= [params.model.A - eye(params.model.nx), params.model.B;
                params.model.C_ref * params.model.C, zeros(size(params.model.C_ref * params.model.C,1), params.model.nu)];
b_steady_state = [-params.model.Bd * d;params.exercise.T_ref - params.model.C_ref*params.model.Cd*d];
out = A_steady_state\b_steady_state;
x_s = out(1:params.model.nx);
u_s = out(params.model.nx+1:end);

end
