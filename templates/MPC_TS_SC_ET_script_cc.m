%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2025, Amon Lahr, Simon Muntwiler, Antoine Leeman, Fabian Flürenbrock & Marco Heim Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% ADD STUFF HERE
Q = diag(params.exercise.QdiagOpt);
R = diag(params.exercise.RdiagOpt);
N = 30;
S = 500*eye(size(params.constraints.StateMatrix,1));
v=  50000;
[H,h] = lqr_maxPI(Q,R,params);
S_t= 500*eye(size(H,1));
v_t= 50000;
soft_MPC = MPC_TS_SC_ET(Q, R, N, H, h, S, v, S_t, v_t, params);
MPC = MPC_TS(Q,R,N,H,h,params);
x0 = params.exercise.InitialConditionB;  
[X,U, ctrl_info] = simulate(x0,MPC, params);
[X_soft,U_soft, ctrl_info_soft] = simulate(x0,soft_MPC, params);

%% Save
current_folder = fileparts(which(mfilename));
save(fullfile(current_folder, "MPC_TS_SC_ET_script_cc.mat"), 'v', 'S', 'v_t', 'S_t');

