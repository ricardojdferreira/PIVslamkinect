function rgbd = get_rgbd_new(xyz, rgb, R, T, K_rgb)
%get_rgbd_new recebe os pontos xyz da camara ir , a rotacao R e translacao T da camara IR para a camara RGB e K_rgb, os parametros intrinsecos da camara rgb

%parametros intrinsecos da camara rgb
Kx = K_rgb(1,1);
Cx = K_rgb(1,3);
Ky = K_rgb(2,2);
Cy = K_rgb(2,3);

%Transformacao dos pontos xyz para o referencial da camara RGB
xyz_rgb = R * xyz';
xyz_rgb = [xyz_rgb(1,:) + T(1); xyz_rgb(2,:) + T(2); xyz_rgb(3,:) + T(3)];

x = xyz_rgb(1,:);
y = xyz_rgb(2,:);
z = xyz_rgb(3,:);

%Projecao dos pontos xyz_rgb na camara rgb
u = round(Kx * x./z + Cx);
v = round(Ky * y./z + Cy);

rgb_size = size(rgb);
n_pixels = numel(rgb(:,:,1));

%filtragem de pontos com correspondencias impossiveis
v(v > rgb_size(1)) = 1;
v(v < 1) = 1;
u(u > rgb_size(2)) = 1;
u(u < 1) = 1;

%Mapeamento dos pontos para as cores respetivas
rgb_inds = sub2ind(rgb_size, v, u);
rgb_aux = reshape(rgb,480*640,3);
rgbd = rgb_aux(rgb_inds,:);
rgbd( xyz(:,3) == 0,:) = 0;
rgbd = double(rgbd);

end
