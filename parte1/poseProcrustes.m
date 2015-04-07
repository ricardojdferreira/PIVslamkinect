function [ R,t ] = poseProcrustes( xy_mkrs,mapa_box,CamMatrix)
% [R,t] = poseProcrustes( xy_mkrs,mapa_box,CamMatrix )
% Estima��o de Pose e R e t com Least Squares no caso n�o-planar

%% LEAST SQUARES
% Cria��o da matriz M
M = zeros(size(xy_mkrs,1)*2,12);
for j = 1:size(xy_mkrs,1)
M(2*j-1,1:4) = [mapa_box(j,:) 1];
M(2*j-1,9:12) = -xy_mkrs(j,1)*[mapa_box(j,:) 1];
M(2*j,5:8) = [mapa_box(j,:) 1];
M(2*j,9:12) = -xy_mkrs(j,2)*[mapa_box(j,:) 1];
end

% SVD na matriz M
[~,~,V] = svd(M);

% Matriz P aproximada (vector pr�prio associado ao valor pr�prio menor)
Ptilda = [V(1:4,end) V(5:8,end) V(9:12,end)]';

%% PROCRUSTES
% Matriz [Q|q]
Qq = CamMatrix\Ptilda;
Q = Qq(:,1:3);
q = Qq(:,4);

% Determina��o de alpha
alpha = abs(sqrt(3))/norm(Q,'fro');
if det(Q) < 0
    alpha = -alpha;
end

% SVD e determina��o de R e t
[U,~,V2] = svd(alpha*Q);
Sigma = eye(3);
Sigma(3,3) = sign(det(U*V2'));

R = U*Sigma*V2';
t = alpha*q;

end


