function [H_lqr, h_lqr] = lqr_maxPI(Q, R, params)
  % unpack
  A  = params.model.A;
  B  = params.model.B;
  Hx = params.constraints.StateMatrix;
  hx = params.constraints.StateRHS;
  Hu = params.constraints.InputMatrix;
  hu = params.constraints.InputRHS;

  % design LQR
  lqr_ctrl = LQR(Q,R,params);
  K       = -dlqr(A,B,Q,R);            % so u = -K*x
  Acl     = A + B*K;                % closed-loop dynamics

  % build the one-step constraint polyhedron X0 = { x | Hx x <= hx, Hu(-K)x <= hu }
  H0 = [Hx;  Hu*K];
  h0 = [hx;  hu];
  X0 = Polyhedron('A',H0,'b',h0);

  % wrap as an autonomous LTISystem
  sys_cl = LTISystem('A',Acl);  

  % tell it “every x must lie in X0 at all times”
  sys_cl.x.with('setConstraint');    % activate the setConstraint filter
  sys_cl.x.setConstraint = X0;       % assign your polyhedron

  % compute the maximal positively-invariant set in one call
  Xinv = sys_cl.invariantSet();      % <— MPT3 does the Pre ∩ repeat C++ loop for you :contentReference[oaicite:0]{index=0}

  % pull out the H-representation
  H_lqr = Xinv.A;
  h_lqr = Xinv.b;
end
