function plotPointcloud( pcloud )
% plotPointcloud( pcloud )
% Faz plot da point cloud
figure('name','Point Cloud');
for k = 1:size(pcloud,1)
    plot3(pcloud(k,1),pcloud(k,2),pcloud(k,3),'.','Color',pcloud(k,4:6)/255);
    hold on
end
end