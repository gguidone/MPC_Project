%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2025, Amon Lahr, Simon Muntwiler, Antoine Leeman, Fabian Flürenbrock & Marco Heim Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [A, B, B_d] = discretize_system_dist(Ac, Bc, Bd_c, params)
B_t = [Bc, Bd_c];
c_sys = ss(Ac,B_t,zeros(1,size(Ac,2)),zeros(1,size(B_t,2)));
d_sys = c2d(c_sys, params.model.TimeStep, 'zoh');
A = d_sys.A;
B_tot = d_sys.B;
B = B_tot(:, 1:size(Bc,2));
B_d = B_tot(:, size(Bc,2)+1:end);
end