Z = 1 - [Z1 Z2];
Z = double(Z);
E = (Z/max(max(Z)));

E = double(E);

X = E';
%X = Z';


save dados_rede.mat X Thetam Gamam;