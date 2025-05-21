%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2025, Amon Lahr, Simon Muntwiler, Antoine Leeman, Fabian Flürenbrock & Marco Heim Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% CONTINUE BELOW THIS LINE
%clear all
%params = generate_params_cc();
%params = generate_params_delta_cc(params);

Q = diag([4700,5000,10]);
R = diag([0.07,0.007]);

controller = LQR(Q,R, params);
[X, U] = simulate(params.exercise.InitialConditionA, controller, params);

x_s = params.exercise.x_s;
u_s = params.exercise.u_s;
[T1_max, T2_min, T2_max, P1_min, P1_max, P2_min, P2_max, input_cost, cstr_viol] = traj_constraints_cc(X + x_s, U + u_s, params);

fprintf('Constraint violation: %f\n\n', cstr_viol);

fprintf('Final trajectory values:\n');
fprintf('  |X(1,30)| = %.4f\n', abs(X(1,30)));
fprintf('  |X(2,30)| = %.4f\n', abs(X(2,30)));
fprintf('  |X(1,60)| = %.4f\n', abs(X(1,60)));
fprintf('  |X(2,60)| = %.4f\n\n', abs(X(2,60)));

fprintf('Trajectory constraints:\n');
fprintf('  T1_max  = %.4f\n', T1_max);
fprintf('  T2_min  = %.4f\n', T2_min);
fprintf('  T2_max  = %.4f\n', T2_max);
fprintf('  P1_min  = %.4f\n', P1_min);
fprintf('  P1_max  = %.4f\n', P1_max);
fprintf('  P2_min  = %.4f\n', P2_min);
fprintf('  P2_max  = %.4f\n', P2_max);
fprintf('  Input cost = %.4f\n', input_cost);


%% Save results
current_folder = fileparts(which(mfilename));
save(fullfile(current_folder, "lqr_tuning_script_cc.mat"), 'Q', 'R', 'X', 'U');