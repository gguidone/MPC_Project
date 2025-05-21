%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2025, Amon Lahr, Simon Muntwiler, Antoine Leeman, Fabian Flürenbrock & Marco Heim Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function Wsim = generate_disturbances_cc(params_robust)
   Hw  = params_robust.constraints.DisturbanceMatrix;  % = [ I; -I ]
   hw  = params_robust.constraints.DisturbanceRHS;     % = [ Δw_max; -Δw_min ]
   N_sim = params_robust.exercise.SimHorizon;
   M = N_sim;
   n_w = size(Hw,2);
   % 1) Vertici (requsisito: MPT3)
P    = Polyhedron('A',Hw,'b',hw);
V    = P.V;
if isempty(V)
    error('Polytopo vuoto o non limitato');
end

% 2) Facets e centroide
facets   = convhulln(V);
centroid = mean(V,1);

% 3) Semplici
nVerts   = size(V,1);
V_all    = [V; centroid];
simplices = [facets, repmat(nVerts+1, size(facets,1), 1)];

% 4) Volumi proporzionali
dim   = size(V,2);
vols  = zeros(size(simplices,1),1);
for i = 1:numel(vols)
    verts = V_all(simplices(i,:),:);
    Mmat  = (verts(1:end-1,:) - verts(end,:))';
    vols(i) = abs(det(Mmat));
end
vols = vols / sum(vols);

% 5) Selezione del semplice
cdf = [0; cumsum(vols)];
r   = rand(M,1);
[~,~,simpIdx] = histcounts(r, cdf);

% 6) Campionamento in ciascun semplice
W = zeros(dim, M);
for j = 1:M
    verts = V_all(simplices(simpIdx(j),:),:);
    e = -log(rand(dim+1,1));
    b = e / sum(e);
    W(:,j) = b' * verts;
end
Wsim = W;

end