function [R,T] = Procrustes2(X1,X2)
%Função que recebe duas matriz com coordenadas x y e z 
%representando 2 imagens e que obtem a transformação entre elas
% (rotação R e translação T) resolvendo o problema de procrustes

X1med(1,:) = mean(X1(1,:));
X1med(2,:) = mean(X1(2,:));
X1med(3,:) = mean(X1(3,:));

X2med(1,:) = mean(X2(1,:));
X2med(2,:) = mean(X2(2,:));
X2med(3,:) = mean(X2(3,:));

X1c(1,:)=X1(1,:)-X1med(1,:);
X1c(2,:)=X1(2,:)-X1med(2,:);
X1c(3,:)=X1(3,:)-X1med(3,:);

X2c(1,:)=X2(1,:)-X2med(1,:);
X2c(2,:)=X2(2,:)-X2med(2,:);
X2c(3,:)=X2(3,:)-X2med(3,:);

%Procrustes
[U,~,V] =svd(X1c*X2c');%single value decomposition

Sigma = eye(3);
Sigma(3,3) = sign(det(U*V'));

%matriz rotação
R = U*Sigma*V';
%vector translação
T = X1med - R*X2med;

end

