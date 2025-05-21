%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2025, Amon Lahr, Simon Muntwiler, Antoine Leeman, Fabian Flürenbrock & Marco Heim Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function params_robust_tube = compute_tightening(K_tube,H_tube,h_tube,params_robust)  
	% YOUR CODE HERE
    params_robust_tube = params_robust;
    Hx = params_robust.constraints.StateMatrix;
    hx = params_robust.constraints.StateRHS; 
    Hu = params_robust.constraints.InputMatrix;
    hu = params_robust.constraints.InputRHS;
    X = Polyhedron('A', Hx, 'b', hx);
    U = Polyhedron('A', Hu, 'b', hu);
    E = Polyhedron('A', H_tube, 'b', h_tube);
    X_E = X - E;
    U_E = U - K_tube*E;
    params_robust_tube.constraints.StateMatrix = X_E.A;
    params_robust_tube.constraints.StateRHS =  X_E.b;
    params_robust_tube.constraints.InputMatrix =  U_E.A;
    params_robust_tube.constraints.InputRHS = U_E.b; 
end