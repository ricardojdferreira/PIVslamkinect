function [ pcloudvoxelized ] = voxelize( pcloud,voxelsize )
% [ pcloudvoxelized ] = voxelize( pcloud,voxelsize )
% Realiza a voxeliza��o da point cloud obtida

% M�todo para discriminar os voxels
pcloud1 = fix(pcloud(:,1:3)/voxelsize)*voxelsize;

% Retira os �nicos pontos que n�o se repetem e faz an�lise das cores
[paux,ia,~] = unique(pcloud1(:,1:3),'rows');
disp(['A processar ' num2str(length(paux)) ' pontos']);
pcloudvoxelized = [paux pcloud(ia,4:6)];

end