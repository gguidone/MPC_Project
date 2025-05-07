%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2025, Amon Lahr, Simon Muntwiler, Antoine Leeman, Fabian Flürenbrock & Marco Heim Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

classdef MPC_TS_SC_ET
    properties
        yalmip_optimizer
    end

    methods
        function obj = MPC_TS_SC_ET(Q, R, N, H, h, S, v, S_t, v_t, params)
            
            % ADD STUFF HERE
            A = params.model.A;
            B = params.model.B;
            Hx = params.constraints.StateMatrix;
            hx = params.constraints.StateRHS;
            Hu = params.constraints.InputMatrix;
            hu = params.constraints.InputRHS;
            P = dare(A,B,Q,R);
            U = sdpvar(repmat(params.model.nu,1,N),repmat(1,1,N));
            x0 = sdpvar(params.model.nx,1);
            X = sdpvar(repmat(params.model.nx,1,N+1),repmat(1,1,N+1));
            E = sdpvar(repmat(size(S,1),1,N),repmat(1,1,N));
            e_t = sdpvar(repmat(size(S_t,1),1,1),repmat(1,1,1));
            constraints = [X{1} == x0];
            objective = 0;
            for k = 1:N
             objective = objective + X{k}'*Q*X{k} + U{k}'*R*U{k} + E{k}'*S*E{k} + v * norm(E{k},1);
             constraints = [constraints, X{k+1} == A*X{k} + B*U{k}];
             constraints = [constraints, Hu *U{k}<= hu, Hx*X{k}<=hx + E{k}, E{k}>=0];
            end
            constraints = [constraints, H*X{N+1}<=h + e_t, e_t>=0];
            objective = objective + X{N+1}'*P*X{N+1}; 
            objective = objective + e_t'*S_t*e_t + v_t * norm(e_t,1);
            opts = sdpsettings('verbose', 1, 'solver', 'quadprog', 'quadprog.TolFun', 1e-8);
            obj.yalmip_optimizer = optimizer(constraints, objective, opts, x0, {U{1} objective});
        end

        function [u, ctrl_info] = eval(obj,x)
            %% evaluate control action by solving MPC problem, e.g.
            tic
            [optimizer_out,errorcode] = obj.yalmip_optimizer(x);
            solvetime = toc;
            [u, objective] = optimizer_out{:};

            feasible = true;
            if (errorcode ~= 0)
                feasible = false;
            end

            ctrl_info = struct('ctrl_feas', feasible, 'objective', objective, 'solvetime', solvetime);
        end
    end
end
