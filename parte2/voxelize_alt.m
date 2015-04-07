function [ pcloudvoxelized ] = voxelize_alt( pcloud,voxelsize )
% [ pcloudvoxelized ] = voxelize( pcloud,voxelsize )
% Realiza a voxelização da point cloud obtida

% Método para discriminar os voxels
pcloud1 = sortrows([fix(pcloud(:,1:3)/voxelsize)*voxelsize pcloud(:,4:6)]);
tag = any(diff(pcloud1(:,1:3),1),2);
paux = unique(pcloud1(:,1:3),'rows');
colors = zeros(size(paux));
aux = pcloud1(1,4:6);
k = 1;
p = 10;
P = size(pcloud1,1);
disp(['A processar ' num2str(length(paux)) ' pontos']);

for i = 1:size(pcloud1,1)-1
    if tag(i) == 0
        aux = [aux;pcloud1(i+1,4:6)];
    end
    if tag(i) == 1
        colors(k,:) = median(aux);
        k = k+1;
        aux = pcloud1(i+1,4:6);
    end
    if (i/P)*100 > p
        disp(['Foram filtrados ' num2str(p) '% dos pontos']);
        p = p+10;
    end
end

pcloudvoxelized = [paux colors];

end