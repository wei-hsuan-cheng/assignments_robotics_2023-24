%% B FK
clear; clc; close all;

addpath('C:\Users\RMML05\assignments_robotics_2023-24\ModernRobotics-master\packages\MATLAB\mr')

load('DHtable');

syms th1 th2 th3 th4 th5
th1 = 0; th2 = 0; th3 = 0; th4 = 0; th5 = 0;
theta = [th1, th2, th3, th4, th5].';

T05 = eye(4);
% alpha -> a -> d -> theta
for i = 1 : 5
	T(:,:,i) = DH2SE3(alpha(i),a(i),d(i),theta(i)); % ^{i-1}_iT
	T05 = T05 * T(:,:,i);
end