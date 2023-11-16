%% C-1 IK- orientation problem
clear; clc; close all;

addpath('C:\Users\RMML05\assignments_robotics_2023-24\ModernRobotics-master\packages\MATLAB\mr')

load('DHtable');

syms th4 alph4 th5

R34 = [cos(th4) -sin(th4) 0;
       sin(th4) cos(th4) 0;
       0 0 1];

R45 = [1 0 0;
       0 cos(alph4) -sin(alph4);
       0 sin(alph4) cos(alph4)] * [
       cos(th5) -sin(th5) 0;
       sin(th5) cos(th5) 0;
       0 0 1];

R35 = R34 * R45;
