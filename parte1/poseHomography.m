function [ R,t] = poseHomography( xy_mkrs,mapa_box,ind,mapa_RT,CamMatrix )
% [R,t] = poseHomography( xy_mkrs,mapa_box,ind,mapa_RT,CamMatrix )
% Estima��o de homografia e R e t no caso coplanar

%% LEAST SQUARES
% Cria��o da matriz M e transforma��o para o referencial da face visivel
M = zeros(size(xy_mkrs,1)*2,9);
mapa = zeros(size(mapa_box));
for j = 1:size(xy_mkrs,1)
    
    % Transforma��o para o referencial da face visivel
    mapa(j,:) = mapa_RT{ind(1)-1}.R'*mapa_box(j,:)'-(mapa_RT{ind(1)-1}.R'*mapa_RT{ind(1)-1}.T);
    
    M(2*j-1,1:3) = [mapa(j,1) mapa(j,3) 1];
    M(2*j-1,7:9) = -xy_mkrs(j,1)*[mapa(j,1) mapa(j,3) 1];
    M(2*j,4:6) = [mapa(j,1) mapa(j,3) 1];
    M(2*j,7:9) = -xy_mkrs(j,2)*[mapa(j,1) mapa(j,3) 1];
end

% SVD na matriz M
[~,~,V] = svd(M);

% Matriz H aproximada
Htilda = [V(1:3,end) V(4:6,end) V(7:9,end)]';
const = sign(Htilda(3,3));
if const == -1
    Htilda = -Htilda;
end

% Factor de normaliza��o
Htilda = CamMatrix\Htilda;
Q = Htilda(:,1:2);
alpha = sqrt(2)/norm(Q,'fro');
r1 = Q(:,1);
r3 = Q(:,2);
r2 = -cross(r1,r3);
Qh = [r1 r2 r3];
if det(Qh)<0
    alpha = -alpha;
end

%% Procrustes e determina��o de R
[U,~,V2] = svd(alpha*Qh);
Sigma = eye(3);
Sigma(3,3) = sign(det(U*V2'));
Rh = U*Sigma*V2';
R = Rh*mapa_RT{ind(1)-1}.R';

%% ESTIMA��O DE t
th = alpha * Htilda(:,3);
t = (th - R*mapa_RT{ind(1)-1}.T);

end