%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2025, Amon Lahr, Simon Muntwiler, Antoine Leeman, Fabian Flürenbrock & Marco Heim Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% --------- YOUR CODE HERE: -------------
%clear all
%g= generate_params_cc();
%params_delta = generate_params_delta_cc(g);
%params_robust = generate_params_robust_cc(params_delta);
% define Q and R

Q = diag(params_robust.exercise.QdiagOpt);
R = diag(params_robust.exercise.RdiagOpt);
N = params_robust.exercise.SimHorizon;
A = params_robust.model.A;
B = params_robust.model.B;
% Obtain tube controller by pole placement 
K = dlqr(A,B,Q,R);
K_tube = params_robust.exercise.K_tube; 
H_tube = params_robust.exercise.H_tube;
h_tube = params_robust.exercise.h_tube;
E = Polyhedron('A', H_tube, 'b', h_tube);
% Obtain mRPI set
H_w = params_robust.constraints.DisturbanceMatrix;
h_w = params_robust.constraints.DisturbanceRHS;
is_RPI = check_RPI(H_tube, h_tube, H_w, h_w, K_tube, params_robust);
% Compute tightening
params_robust_tube = compute_tightening(K_tube, H_tube, h_tube, params_robust);
% Implement Tube-MPC
[H_N, h_N] = lqr_maxPI(Q+K'*R*K,zeros(2,2), params_robust_tube);
tube_controller = MPC_TUBE(Q,R,N,H_N, h_N, H_tube, h_tube, K_tube, params_robust_tube);
x0 = params_robust_tube.exercise.InitialConditionA;  
[X,U, ctrl_info_tube] = simulate(x0,tube_controller, params_robust_tube);
%% Save
save_name = 'MPC_TUBE_params_cc.mat';
save(save_name, 'K_tube','H_tube','h_tube','H_N','h_N', 'params_robust_tube');     % use these variable names