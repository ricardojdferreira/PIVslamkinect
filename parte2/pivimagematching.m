function [transforms,pcloud,matchesCell] = pivimagematching(image_name,depth_cam,rgb_cam,Rdtrgb,Tdtrgb,varargin)
%pivimagematching é a função que corre a segunda parte do projecto
%Tem como inputs:
%image_name que é um array de estruturas com o nome das
%imagens. cada elemento do array é uma estrutra: image_name[k].depth,
%uma string com o caminho do ficheiro .mat com os dados de profundidade da
% da imagem k; image_name[k].rgb, uma string com o caminho do ficheiro jpeg
% da imagem rgb k;
%depth_cam: estrutra com os parâmetros intrinsicos da camara de
%profundidade, nomeadamente a matriz 3x3 depth_cam.K e o array depth_cam.DistCoef
%com os coeficientes de distorção da lente;
%rgb_cam: estrutura com os parâmetros intrinsicos da camara rgb, nomeadamente a matriz 3x3 depth_cam.K e o array depth_cam.DistCoef
%com os coeficientes de distorção da lente;
%Rdtrgb (matriz de rotação 3x3) e Tdrgb (vector de translação 3x1) que
%representam a transformação do sistema de coordenadas da camera de
%profundidade para o sistema de coordenadas da camera de rgb.

%Variáveis opcionais:
%   'voxel_alt',num - voxelização (com método alternativo) e valor numérico do tamanho do voxel;
%   'pcd' - criação de um ficheiro .pcd para usar no PCL viewer;
%   'cc' - criação de um ficheiro para usar no Cloud Compare;
%   'ccimage' - criação de um ficheiro de cada imagem para usar no Cloud Compare;
%   'cloud' - faz o plot da point cloud
%   'threshold' - define o limiar usado pelo vl_feat para fazer
%   correspondencias

%tem como outputs:
%pcloud [Xi Yi Zi Ri Gi Bi], matriz Nx6 com os pontos 3D e a cor associada
%a cada ponto, no referencial do mundo

%transforms, um array de estruturas com o mesmo tamanho do input
%image_name, onde cada elemento contem a transformação entre o sistema de
%referencia da camera de profundidade e o referencial do mundo para a
%imagem k, com os campos: transforms[k].R (matrix de rotação entre a camera
%de profundidade e o referencial do mundo para a imagem k) e
%transforms[k].T (vector translação entre a camera de profundidade e o
%referencial do mundo para a imagem k)
%matchesCell, uma cell com os matches entre as imagens analisadas


%Análise dos inputs
pcd = 0;
voxel_alt = 0;
cloud = 0;
cc = 0;
ccimage = 0;
met = 0;
thresh = 4;

for arg = 1:length(varargin)
    switch varargin{arg}
        case 'pcd'
            pcd = 1;
            pcdfile = ['parte2\pointcloud'  datestr(now,30)];
        case 'cc'
            cc = 1;
            ccfile = ['parte2\pointcloud' datestr(now,30) '.txt'];
        case 'ccimage'
            ccimage = 1;
        case 'voxel_alt'
            voxel_alt = 1;
            voxelsize = varargin{arg+1};
        case 'cloud'
            cloud = 1;
        case 'epnp'
            met = 1;
        case 'threshold'
            thresh = varargin{arg+1};
    end
end

K_ir = depth_cam.K;
K_rgb = rgb_cam.K;

%inicialização de variáveis
transformsC = cell(1,size(image_name,2)-1);
MatchingCell = cell(1,size(image_name,2));
transforms = cell(1,size(image_name,2)-1);
Rotations = cell(1,size(image_name,2)-1);
Translations = cell(1,size(image_name,2)-1);
pcloud = [];
pcloudCell = cell(1,size(transforms,2));

%leitura de imagens e aplicação da SIFT
for i=1:size(image_name,2)
    
    
    rgb = image_name(1,i).rgb;
    rgb=imread(rgb);
    cor = rgb;
    rgb = rgb2gray(im2single(rgb));
    depth  = load(image_name(1,i).depth);
    depth = depth.depth_array;
    xyz = true_points(depth,cor,K_ir);
    xyzcor = get_rgbd_new(xyz,cor,Rdtrgb,Tdtrgb,K_rgb);
    disp(['A fazer SIFT' num2str(i) '...']);
    [f,d] = vl_sift(rgb);
    MatchingStruct.cor = cor;
    MatchingStruct.depth = depth;
    MatchingStruct.f = f;
    MatchingStruct.d = d;
    MatchingStruct.xyz = xyz;
    MatchingStruct.xyzcor = xyzcor;
    MatchingCell{1,i} = MatchingStruct;
    
    
end

%Calculo das rotações e translações entre 2 imagens consecutivas
matchesCell = cell(1,size(image_name,2)-1);

for i=1:(size(image_name,2)-1)
    
    disp(['A fazer match' num2str(i) '...']);
    [matches,~] = vl_ubcmatch(MatchingCell{1,i}.d,MatchingCell{1,i+1}.d,thresh);
    matchesCell{1,i}=matches;
    
    X1 = round(MatchingCell{1,i}.f(1:2,matches(1,:)));
    X2 = round(MatchingCell{1,i+1}.f(1:2,matches(2,:)));
    matches = [];
    %Construção das matrizes 3D
    X1 = get_3D(X1,MatchingCell{1,i}.depth,MatchingCell{1,i}.cor,K_ir,K_rgb,Rdtrgb,Tdtrgb);
    X2 = get_3D(X2,MatchingCell{1,i+1}.depth,MatchingCell{1,i+1}.cor,K_ir,K_rgb,Rdtrgb,Tdtrgb);
    
    [X1,X2]=filter_3D(double(X1),double(X2));
    
    %estimação da transformação (rotação e translação)
    [R,T] = Procrustes2(X1,X2);
    transformC.R =R;
    transformC.T = T;
    transformsC{1,i} = transformC;
    
end
disp('Concluído!')
%Calculo das rotações e translações para o referencial da imagem 1
disp(['A calcular as transformações para referencial de referência']);
Rotations{1,1}=transformsC{1,1}.R;
R = Rotations{1,1};

for i=2:(size(transformsC,2))
    %calcula a rotação
    R = R*transformsC{1,i}.R;
    Rotations{1,i} =R;
end

translacoes =  cell(1,size(transforms,2)-1);
translacoes{1,1} = transformsC{1,1};

for i = 2:size(transforms,2)
    t= Rotations{1,i-1}*transformsC{1,i}.T;
    translacao.t = t;
    translacoes{1,i} = translacao;
end


Translations{1,1} = transformsC{1,1}.T;
Translacao = Translations{1,1};

for i=2:size(translacoes,2)
    
    Translacao = Translacao+translacoes{1,i}.t;
    Translations{1,i}=Translacao;
    
end
for i=1:size(transforms,2)
    
    transformation.R = Rotations{1,i};
    transformation.T = Translations{1,i};
    transforms{1,i} = transformation;
    
end

%Construção da point cloud
for i=1:(size(pcloudCell,2))
    xyzref = transforms{1,i}.R*(MatchingCell{1,i+1}.xyz)'+repmat(transforms{1,i}.T,1,size(MatchingCell{1,i+1}.xyz,1));
    ref.xyzref = xyzref;
    ref.cor = MatchingCell{1,i+1}.xyzcor;
    pcloudCell{1,i} = ref;
end
pcloud = [(MatchingCell{1,1}.xyz)'; (MatchingCell{1,1}.xyzcor)'];
for i=1:(size(pcloudCell,2))
    pcloud = cat(2, pcloud,[pcloudCell{1,i}.xyzref; (pcloudCell{1,i}.cor)']);
end

pcloud = pcloud';
disp('Concluído!')

%Voxelização
if voxel_alt ~= 0
    disp('A voxelizar...')
    pcloudvoxelized = voxelize_alt(pcloud,voxelsize);
    clear pcloud;
    pcloud = pcloudvoxelized;
    disp('Concluído!')
end

% Grava ficheiro para leitura em Cloud Compare
if cc ~= 0
    disp('A gravar ficheiro...')
    dlmwrite('pcloud.txt',pcloudvoxelized);
    disp('Concluído!')
end

% Grava um ficheiro .pcd
if pcd ~= 0
    disp('A criar o ficheiro .pcd...')
    savepcd(pcdfile,pcloud');
    disp('Ficheiro criado!')
end

% Faz plot da point cloud
if cloud ~= 0
    disp('A fazer o "plot"...')
    plotPointcloud(pcloud);
    disp('Concluído!')
end
end