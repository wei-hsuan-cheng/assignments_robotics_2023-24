%%%%%%%%%% Units
d2r = pi / 180;
r2d = 1 / d2r;

%%%%%%%%%% D-H table
alpha0 = 0; alpha1 = pi/2; alpha2 = 0; alpha3 = 0; alpha4 = pi/2; 
alpha = [alpha0, alpha1, alpha2, alpha3, alpha4];
a0 = 0; a1 = 50; a2 = 300; a3 = 350; a4 = 0; 
a = [a0, a1, a2, a3, a4].';
d1 = 358.5; d2 = -35.3; d3 = 0; d4 = 0; d5 = 251;
d = [d1, d2, d3, d4, d5].';

save('DHtable', ...
     'd2r', 'r2d', ...
     'alpha0', 'alpha1', 'alpha2', 'alpha3', 'alpha4', 'alpha', ...
     'a0', 'a1', 'a2', 'a3', 'a4', 'a', ...
     'd1', 'd2', 'd3', 'd4', 'd5', 'd');