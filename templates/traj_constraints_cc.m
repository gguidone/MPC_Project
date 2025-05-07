%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2025, Amon Lahr, Simon Muntwiler, Antoine Leeman, Fabian Flürenbrock & Marco Heim Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [T1_max, T2_min, T2_max, P1_min, P1_max, P2_min, P2_max, input_cost, cstr_viol] = traj_constraints_cc(Xsim, Usim, params)
    T1_max = max(Xsim(1,:));
    T2_max = max(Xsim(2,:));
   
    T2_min = min(Xsim(2,:));

    P1_min  = min(Usim(1,:));
    P1_max = max(Usim(1,:));

    P2_min  = min(Usim(2,:));
    P2_max = max(Usim(2,:));
    
    input_cost = 0;
    for i_time = 1:size(Usim,2)
        input_cost = input_cost + Usim(:,i_time)'*Usim(:,i_time);
    end
    cstr_viol = false;
    if(T1_max >-15 || T2_max>4  || T2_min <0  || P1_max > 0 || P1_min < -2500 || P2_max > 0 || P2_min < -2000)
        cstr_viol = true;
    end
  
end

