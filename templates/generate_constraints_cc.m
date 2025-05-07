%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2025, Amon Lahr, Simon Muntwiler, Antoine Leeman, Fabian Flürenbrock & Marco Heim Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [H_u, h_u, H_x, h_x] = generate_constraints_cc(params)

H_x = [1,0,0;
        -1,0,0;
        0,1,0;
        0,-1,0;
        0,0,1;
        0,0,-1];
T1_max = params.constraints.T1Max;
T1_min = params.constraints.T1Min;
T2_max = params.constraints.T2Max;
T2_min = params.constraints.T2Min;
T3_max = params.constraints.T3Max;
T3_min = params.constraints.T3Min;
h_x = [T1_max;-T1_min;T2_max;-T2_min;T3_max;-T3_min];
H_u = [1,0;
      -1,0;
       0,1;
      0,-1];
p1_max = params.constraints.P1Max;
p1_min = params.constraints.P1Min;
p2_max = params.constraints.P2Max;
p2_min = params.constraints.P2Min;
h_u = [p1_max;-p1_min;p2_max;-p2_min];
end

