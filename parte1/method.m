function [ ind_aux2] = method( ind )
% [ ind_aux2] = method( ind )
% Permite distiguir as quando os pontos são apenas de uma face ou se são de
% mais do que uma face
ind_aux = [1 1 1 2 2 2 3 3 3 4 4 4];
ind_aux2 = zeros(length(ind),1);
for i2 = 1:length(ind)
    ind_aux2(i2) = ind_aux(ind(i2)-1);
end
end

