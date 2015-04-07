function [pcloud,transforms,bproj]=pivwitharuco( image_name, visible_aruco, aruco_cam, depth_cam, rgb_cam,Rdtrgb,Tdtrgb,Rcalib,Tcalib, mapa_RT,varargin)
% [pcloud,transforms,bproj]=pivwitharuco( image_name, visible_aruco, aruco_cam, depth_cam, rgb_cam,Rdtrgb,Tdtrgb,Rcalib,Tcalib, mapa_RT)
% Como variáveis opcionais pode-se ter:
%   'backproj' - realiza o plot da backprojection e o erro associado na
%   estimação da pose do Aruco;
%   'voxel',num - voxelização e valor numérico do tamanho do voxel;
%   'voxel_alt',num - voxelização (com método alternativo) e valor numérico do tamanho do voxel;
%   'pcd' - criação de um ficheiro .pcd para usar no PCL viewer;
%   'cc' - criação de um ficheiro para usar no Cloud Compare;
%   'ccimage' - criação de um ficheiro de cada imagem para usar no Cloud Compare;
%   'cloud' - faz o plot da point cloud
%   'epnp' - usa o algoritmo EPnP para estimação da pose do Aruco

% Análise do input
pcd = 0;
backproj = 0;
bproj=[];
bb=1;
voxel = 0;
voxel_alt = 0;
cloud = 0;
cc = 0;
ccimage = 0;
met = 0;
for arg = 1:length(varargin)
    switch varargin{arg}
        case 'pcd'
            pcd = 1;
            pcdfile = ['parte1\pointcloud'  datestr(now,30)];
        case 'cc'
            cc = 1;
            ccfile = ['parte1\pointcloud' datestr(now,30) '.txt'];
        case 'ccimage'
            ccimage = 1;
        case 'backproj'
            backproj = 1;
        case 'voxel'
            voxel = 1;
            voxelsize = varargin{arg+1};
        case 'voxel_alt'
            voxel_alt = 1;
            voxelsize = varargin{arg+1};
        case 'cloud'
            cloud = 1;
        case 'epnp'
            met = 1;
    end
end

pcloud=[];

% Fazer o loop por todas as imagens
for i = 1:size(image_name,2)
    disp(['A processar a imagem ' num2str(i) '...']);
    if ~isempty(image_name(i))
        
        % ARUCO
        switch met
            case 0
                % Array auxiliar para distinguir homografias
                aux = method(visible_aruco(i).ind);
                
                if range(aux) ~= 0
                    % Não-coplanar
                    [R_aruco,t_aruco] = poseProcrustes(visible_aruco(i).xymrks,visible_aruco(i).xyzbase,aruco_cam.K);
                else
                    % Coplanar
                    [R_aruco,t_aruco] = poseHomography(visible_aruco(i).xymrks,visible_aruco(i).xyzbase,visible_aruco(i).ind,mapa_RT,aruco_cam.K);
                end
            case 1
                % Uso do EPnP
                [R_aruco,t_aruco,~,~]=efficient_pnp_gauss(visible_aruco(i).xyzbase,visible_aruco(i).xymrks,aruco_cam.K);
        end
        
        
        % RGB e IR
        % Carrega as imagens
        RGB = imread(image_name(i).rgb);
        load(image_name(i).depth,'depth_array');
        IR = depth_array;
        
        % Backprojection
        if backproj ~= 0
            bproj(bb)= plotBackprojection(visible_aruco(i).xymrks,visible_aruco(i).xyzbase,R_aruco,t_aruco,aruco_cam.K);
            bb = bb+1;
        end
        
        % Constrói os pontos e as cores
        pontos_xyz=true_points(IR,RGB,depth_cam.K);
        pontos_rgb = get_rgbd_new(pontos_xyz,RGB, Rdtrgb, Tdtrgb, rgb_cam.K);
        
        % Constrói as transformações do referencial da camera de profundidade para
        % o referencial do mundo
        R = R_aruco*Rcalib;
        T = R_aruco*Tcalib+t_aruco;
        pontos_xyz_mundo = R*(pontos_xyz')+repmat(T,1,size(pontos_xyz,1));
        pcloud=[pcloud; [pontos_xyz_mundo' pontos_rgb]];
        transforms(i).R = R;
        transforms(i).T = T;
        
        if ccimage ~= 0
            disp(['A criar o ficheiro Cloud Compare para a imagem ' num2str(i) '...']);
            dlmwrite(['parte1\pointcloudimage' num2str(i) '.txt'],[pontos_xyz_mundo' pontos_rgb]);
            disp('Ficheiro criado');
        end
    end
end

% Criação de voxel com dimensão voxelsize
if voxel ~= 0
    disp('A voxelizar...')
    pcloudvoxelized = voxelize(pcloud,voxelsize);
    pcloud = pcloudvoxelized;
    disp('Concluído!')
end

if voxel_alt ~= 0
    disp('A voxelizar...')
    pcloudvoxelized = voxelize_alt(pcloud,voxelsize);
    pcloud = pcloudvoxelized;
    disp('Concluído!')
end

% Grava um ficheiro .pcd
if pcd ~= 0
    disp('A criar o ficheiro .pcd...')
    savepcd(pcdfile,pcloud');
    disp('Ficheiro criado!')
end

% Grava um ficheiro Cloud Compare
if cc ~= 0
    disp('A criar o ficheiro Cloud Compare...');
    dlmwrite(ccfile,pcloud);
    disp('Ficheiro criado');
end

% Faz plot da point cloud
if cloud ~= 0
    disp('A fazer o "plot"...')
    plotPointcloud(pcloud);
    disp('Concluído!')
end