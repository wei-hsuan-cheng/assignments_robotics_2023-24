clear; clc; close all;

addpath('C:\Users\RMML05\Desktop\assignments_robotics_2023-24\ModernRobotics-master\packages\MATLAB\mr')

%% A-1
clear; clc; close all;
%%%%%%%%%% Units
d2r = pi / 180;
r2d = 1 / d2r;

%%%%%%%%%% u, phi to R
theta = 90 * d2r;
omg = [1 5 2].'; omg = omg / norm(omg);
omg_theta = omg * theta;
so3mat = VecToso3(omg_theta);
R = MatrixExp3(so3mat);

% quat = [cos(theta / 2);
%         omg * sin(theta / 2) - 0.001
%         ];
% 
% R_quat = quat2rotm(quat.');
% 
% if max(abs(R - R_quat)) < 10^(-4)
%     fprintf('Correct\n');
% end

%% A-2
clear; clc; close all;
%%%%%%%%%% Units
d2r = pi / 180;
r2d = 1 / d2r;

%%%%%%%%%% R to u, phi
R = [.911 -.244 .333;
     .333 .911 -.244;
     -.244 .333 .911];
phi = acos((trace(R) - 1) / 2); % rad
phi_d = phi * r2d;
u_hat = (1 / (2 * sin(phi))) * [R(3,2) - R(2,3);
                                R(1,3) - R(3,1);
                                R(2,1) - R(1,2)];

%%%%%%%%%% Check if R stays in the manifold
detR = det(R);
R_transpose = R.';
R_inverse = inv(R);

%% B-1
clear; clc; close all;
%%%%%%%%%% Units
d2r = pi / 180;
r2d = 1 / d2r;

%%%%%%%%%% Homogeneous coordinates
A_BR = rotz(45) * roty(30) * rotx(60);
A_Bp = [3 6 9].';
A_BT = [A_BR A_Bp;
        zeros(1,3) 1];

%% B-2
clear; clc; close all;
%%%%%%%%%% Units
d2r = pi / 180;
r2d = 1 / d2r;

%%%%%%%%%% Rotation matrices
theta1 = 60 * d2r;
omg1 = [1 1 1].'; omg1 = omg1 / norm(omg1);
omg_theta1 = omg1 * theta1;
so3mat1 = VecToso3(omg_theta1);
R1 = MatrixExp3(so3mat1);

theta2 = 30 * d2r;
omg2 = [1 -1 2].'; omg2 = omg2 / norm(omg2);
omg_theta2 = omg2 * theta2;
so3mat2 = VecToso3(omg_theta2);
R2 = MatrixExp3(so3mat2);

theta3 = 45 * d2r;
omg3 = [-1 -3 1].'; omg3 = omg3 / norm(omg3);
omg_theta3 = omg3 * theta3;
so3mat3 = VecToso3(omg_theta3);
R3 = MatrixExp3(so3mat3);

%%%%%%%%%% If rotate about its body-frame at each time
R_body = R1 * R2 * R3;
origin_CT_body = [R_body zeros(3,1);
                  zeros(1,3) 1];

%%%%%%%%%% If rotate about the world-frame at each time
R_world = R3 * R2 * R1;
origin_CT_world = [R_world zeros(3,1);
                  zeros(1,3) 1];