function xyz_ret = true_points(depth_vec,rgb,K_ir)
% depth_vec - matriz da profundidade do kinect
% rgb - matriz da imagem rgb
% K_ir - matriz dos parametros intrinsecos da camara de IR

%% Setup inicial de variaveis uteis

depth_size=size(depth_vec);
depth_vec=reshape(depth_vec,[depth_size(1)*depth_size(2),1]);
rgb_size = size(rgb);
im_size = [rgb_size(1) rgb_size(2)];

%% Parâmetros Intrinsecos da Câmara IR
% Definicao de variaveis de forma a que seja mais facil a manipulacao com
% dados da matriz de parâmetros intrinsecos

fsx_ir = K_ir(1,1); % f.Sx
Cx_ir = K_ir(1,3); % Cx
fsy_ir = K_ir(2,2); % f.Sy
Cy_ir = K_ir(2,3); % Cy

%% Calculo de matrizes u e v com subtracao de Cx/y

u = repmat(1:im_size(2),im_size(1),1);
u = u(:)-Cx_ir;
v = repmat((1:im_size(1))',im_size(2),1);
v=v(:)-Cy_ir;

%% Calculo de todas as coordenadas - x,y,z

z = double(depth_vec)*0.001; % Converte todas as coordenadas para doubles com unidades em metros
x = (z/fsx_ir) .* u ; % calculo de cada coordenada x a partir de cada z e f.Sx
y = (z/fsy_ir) .* v;  % calculo de cada coordenada y a partir de cada z e f.Sy

xyz=[x y z]; % point cloud antes de ser filtrada

%% Filtragem dos pontos com coordenada z<0.001

xyz(~any(xyz,2),:)=[];
xyz_ret=xyz(xyz(:,3)>0.001,:);

% k=0;
% 
% for i=1:length(u)
%     
%     if (xyz(i,3)>0.001)
%         k=k+1;
%     end
%     
% end
% 
% lista=zeros(k,3);
% 
% k=1;
% 
% for r=1:length(u)
%     
%     if (xyz(r,3)>0.001)
%         lista(k,1)=xyz(r,1);
%         lista(k,2)=xyz(r,2);
%         lista(k,3)=xyz(r,3);
%         k=k+1;
%     end
%     
% end

%% DEBUG & FINAL

% plot3(xyz_ret(1:10:end,1),xyz_ret(1:10:end,2),xyz_ret(1:10:end,3),'.');
% pause
% close

end