function meanerror = plotBackprojection( xy_mkrs,xyzbase,R,t,K )
% meanerror = plotBackprojection( xy_mkrs,xyzbase,R,t,K )
% Calcula a retroprojecção após a estimação da pose e o erro quadrático
% médio associado

% Calcula a retroprojecção
xyz = K*[R t]*[xyzbase ones(size(xyzbase,1),1)]';
xyz_proj = zeros(size(xyz,1)-1,size(xyz,2));
xyz_proj(1,:) = xyz(1,:)./xyz(3,:);
xyz_proj(2,:) = xyz(2,:)./xyz(3,:);

% Cálculo do erro
error = abs(xy_mkrs'-xyz_proj);
meanerror = sum(sqrt(error(1,:).^2+error(2,:).^2))/size(error,2);
disp(['Erro médio da retroprojecção: ' num2str(meanerror) ' píxeis'])

% Mostra ambos os conjuntos de pontos
figure();
scatter(xy_mkrs(:,1),xy_mkrs(:,2),'ro');
hold on
scatter(xyz_proj(1,:),xyz_proj(2,:),'b*');
end

