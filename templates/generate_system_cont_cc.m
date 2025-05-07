%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2025, Amon Lahr, Simon Muntwiler, Antoine Leeman, Fabian Flürenbrock & Marco Heim Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [Ac, Bc, Bdc] = generate_system_cont_cc(params)

Bdc = diag([1/params.model.m1, 1/params.model.m2, 1/params.model.m3]);

a10 = params.model.a1o;
a12 = params.model.a12;
a23 = params.model.a23;
a20 = params.model.a2o;
a30 = params.model.a3o;
A = [-a12-a10, a12, 0;
    a12,-a12-a23-a20,a23;
    0, a23, -a23-a30];
B = [1,0;
    0,1;
    0,0];
Ac = Bdc * A;
Bc = Bdc * B;

end