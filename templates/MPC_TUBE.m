%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2025, Amon Lahr, Simon Muntwiler, Antoine Leeman, Fabian Flürenbrock & Marco Heim Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

classdef MPC_TUBE
    properties
        yalmip_optimizer
        K_tube
    end

    methods
        function obj = MPC_TUBE(Q,R,N,H_N,h_N,H_tube,h_tube,K_tube,params_robust_tube)
            obj.K_tube = K_tube;

            % YOUR CODE HERE
            A = params_robust_tube.model.A;
            B = params_robust_tube.model.B;
            Hx = params_robust_tube.constraints.StateMatrix;
            hx = params_robust_tube.constraints.StateRHS;
            Hu = params_robust_tube.constraints.InputMatrix;
            hu = params_robust_tube.constraints.InputRHS;
            P = dare(A,B,Q,R);
            V = sdpvar(repmat(params_robust_tube.model.nu,1,N),repmat(1,1,N));
            x0 = sdpvar(params_robust_tube.model.nx,1);
            Z = sdpvar(repmat(params_robust_tube.model.nx,1,N+1),repmat(1,1,N+1));
            constraints = [H_tube*(x0-Z{1})<=h_tube]; 
            objective = 0; 
            for k = 1:N
             objective = objective + Z{k}'*Q*Z{k} + V{k}'*R*V{k};
             constraints = [constraints, Z{k+1} == A*Z{k} + B*V{k}];
             constraints = [constraints, Hu *V{k}<= hu, Hx*Z{k}<=hx];
            end
            constraints = [constraints, H_N*Z{N+1}<=h_N];
            objective = objective + Z{N+1}'*P*Z{N+1}; 
            opts = sdpsettings('verbose',1,'solver','quadprog');
            obj.yalmip_optimizer = optimizer(constraints,objective,opts,x0,{V{1} Z{1} objective});
        end

        function [u, ctrl_info] = eval(obj,x)
            %% evaluate control action by solving MPC problem, e.g.
            tic;
            [optimizer_out,errorcode] = obj.yalmip_optimizer(x);
            solvetime = toc;
            
            % YOUR CODE HERE
            v_star = optimizer_out{1};
            z_star = optimizer_out{2};
            objective = optimizer_out{3};
            u = v_star + obj.K_tube*(x-z_star);
            feasible = true;
            if (errorcode ~= 0)
                feasible = false;
            end

            ctrl_info = struct('ctrl_feas',feasible,'objective',objective,'solvetime',solvetime);
        end
    end
end