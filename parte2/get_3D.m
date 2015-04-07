function xyz_3D = get_3D(X,depth_vec,rgb,K_ir,K_rgb,R,T)

%obtencao dos pontos totais no referencial da camara IR
xyz=true_points(depth_vec,rgb,K_ir);

%Parametros intrinsecos da camara RGB
fsx = K_rgb(1,1); 
Cx = K_rgb(1,3);
fsy = K_rgb(2,2);
Cy = K_rgb(2,3);

%Transformacao dos pontos para o referencial da camara RGB
xyz_rgb = R * xyz';
xyz_rgb = [xyz_rgb(1,:) + T(1); xyz_rgb(2,:) + T(2); xyz_rgb(3,:) + T(3)];

x = xyz_rgb(1,:);
y = xyz_rgb(2,:);
z = xyz_rgb(3,:);

%Projecao dos pontos na câmara RGB
u = round(fsx * x./z + Cx);
v = round(fsy * y./z + Cy);

rgb_size = size(rgb);

%Filtragem dos pontos obtidos para a projeção
v(v > rgb_size(1)) = rgb_size(1);
v(v < 1) = 1;
u(u > rgb_size(2)) = rgb_size(2);
u(u < 1) = 1;

mat=zeros(rgb_size(1),rgb_size(2),3);

%Construcao de uma matriz tridimensional que relaciona os pontos (u,v) com os pontos (x,y,z) no referencial da camara RGB
for i=1:length(u)
    mat(v(i),u(i),1)=xyz(i,1);
    mat(v(i),u(i),2)=xyz(i,2);
    mat(v(i),u(i),3)=xyz(i,3);
end

%Passagem para coordenadas tridimensionais das features encontradas em coordenadas (u,v) para (x,y,z)
for j = 1:size(X,2)
    ik=X(2,j);
    jk=X(1,j);
    
    X(1,j)=mat(ik,jk,1);
    X(2,j)=mat(ik,jk,2);
    X(3,j)=mat(ik,jk,3);
end

xyz_3D=X;

end