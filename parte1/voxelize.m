function [ pcloudvoxelized ] = voxelize( pcloud,voxelsize )
% [ pcloudvoxelized ] = voxelize( pcloud,voxelsize )
% Realiza a voxelização da point cloud obtida

% Método para discriminar os voxels
pcloud1 = fix(pcloud(:,1:3)/voxelsize)*voxelsize;

% Retira os únicos pontos que não se repetem e faz análise das cores
[paux,ia,~] = unique(pcloud1(:,1:3),'rows');
disp(['A processar ' num2str(length(paux)) ' pontos']);
pcloudvoxelized = [paux pcloud(ia,4:6)];

end