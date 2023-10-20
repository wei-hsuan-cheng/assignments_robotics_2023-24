%% C-3 IK-case 2
clear; clc; close all;

addpath('C:\Users\RMML05\assignments_robotics_2023-24\ModernRobotics-master\packages\MATLAB\mr')

load('DHtable');

fprintf('=====HW2, Part C-3, IK-case 2=====\n');

%%%%%%%%%% Target pose
q_target = [600, 100, 0, -pi/4, 0, 0].';
p05 = q_target(1 : 3);
% R05 = rotx(q_target(6) * r2d) * roty(q_target(5) * r2d) * rotz(q_target(4) * r2d); % fixed angle
R05 = rotz(q_target(4) * r2d) * roty(q_target(5) * r2d) * rotx(q_target(6) * r2d); % euler angle
T05 = [R05, p05;
    zeros(1,3), 1];

%%%%%%%%%% IK- position problem
p04 = p05 + R05 * [0, 0, -d5].';
xw = p04(1); yw = p04(2); zw = p04(3);

%%%%% Defining some useful variables
r = sqrt(xw^2 + yw^2);
zw_prime = zw - d1;
r_prime = sqrt(r^2 - d2^2) + a1; % different from case 1

%%%%% Solve for th1, th2 and th3 (different from case 1)
lambda1 = atan2(yw, xw); lambda1d = lambda1 * r2d;
mu1 = atan2(sqrt(r^2 - d2^2), -d2); mu1d = mu1 * r2d;
lambda2 = atan2(zw_prime, r_prime); lambda2d = lambda2 * r2d;
D2 = (a3^2 - a2^2 - (r_prime^2 + zw_prime^2)) / (2 * a2 * sqrt(r_prime^2 + zw_prime^2));
D3 = (r_prime^2 + zw_prime^2 - a2^2 - a3^2) / (2 * a2 * a3);

th1 = (3/2) * pi + lambda1 - mu1; th1d = th1 * r2d;

if D2 > 1 || D2 < -1 || D3 > 1 || D2 < -1
    fprintf('Position problem: failed.\n');
else
    fprintf('Position problem: succeeded.\n');
    th2 = atan2(sqrt(1 - D2^2), D2) - lambda2; th2d = th2 * r2d;
    th3 = atan2(sqrt(1 - D3^2), D3); th3d = th3 * r2d;
    theta = [th1, th2, th3, NaN, NaN].';

    %%%%%%%%%% IK- orientation problem
    T03 = eye(4);
    for i = 1 : 3
        T(:,:,i) = DH2SE3(alpha(i), a(i), d(i), theta(i));
        T03 = T03 * T(:,:,i);
    end
    R03 = T03(1 : 3, 1 : 3);
    R35 = R03.' * R05;

    %%%%% Solve for th4, th5 (same as case 1)
    if abs(R35(3,3) - 0) > 10^(-4)
        fprintf('Orientation problem: failed.\n');
    else
        fprintf('Orientation problem: succeeded.\n');
        th4 = atan2(R35(1,3), -R35(2,3)); th4d = th4 * r2d;
        th5 = atan2(R35(3,1), R35(3,2)); th5d = th5 * r2d;
        theta = [th1, th2, th3, th4, th5].';

        %%%%%%%%%% Verify IK using FK
        T05_test = eye(4);
        % alpha -> a -> d -> theta
        for i = 1 : 5
        	T(:,:,i) = DH2SE3(alpha(i),a(i),d(i),theta(i)); % ^{i-1}_iT
        	T05_test = T05_test * T(:,:,i);
        end

        if norm( se3ToVec(MatrixLog6(T05_test)) - se3ToVec(MatrixLog6(T05)) ) < 10^(-3)
            IK_test = 'succeeded';
        else
            IK_test = 'failed';
        end
        fprintf(sprintf('Inverse kinematics: %s.\n',IK_test));

        %%%%%%%%%% Verify constrained IK
        if th2 + th3 <= 2 * pi || th2 + th3 > pi
            elbow_config = 'true';
        else
            elbow_config = 'false';
        end
        fprintf(sprintf('Elbow-up configuration: %s.\n',elbow_config));

        if abs(th2 + th3 + th4 - pi) < 10^(-3)
            tip_downward = 'true';
        else
            tip_downward = 'false';
        end
        fprintf(sprintf('Gripper tip vertically downward: %s.\n',tip_downward));

    end

end

