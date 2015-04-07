function [X1new,X2new] = filter_3D(X1,X2)
    %Recebe X1 e X2 e devolve X1 e X2 filtrados
bonsinds1=[];

bonsinds2=[];

%Posicoes das features com coordenadas plausiveis ( com z!=0)
for i=1:length(X1) 
    if X1(3,i)~=0
        bonsinds1=[bonsinds1 i];
    end
    
    if X2(3,i)~=0
        bonsinds2=[bonsinds2 i];
    end
end

%Intersecao das posicoes dos features em que existem features plausiveis
bonsinds=intersect(bonsinds1,bonsinds2);

X1new=[];
X2new=[];

%Construcao das novas listas X1 e X2 com features apagados
for i=1:length(bonsinds)
   X1new=[X1new X1(:,bonsinds(i))];
   X2new=[X2new X2(:,bonsinds(i))];
end

end
