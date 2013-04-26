%Main Planner %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%A program by Edoardo Cacciavillani %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%MPLF %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all
close all
clc
tic
%Data input %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
pos=[110,60]; %% should be input from odometry + SLAM
load('BEL.mat'); %% should be input from SLAM
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

[goalx,goaly,found]=NBV(BEL,pos);
if found==1 % start path search only if we have a goal
goal=[goalx,goaly];
Mapp=MaPP(BEL);
[done,PATH]=AES(Mapp,pos,goal);
else
Path=0;
end
toc

%NOTE in terms of time the first one to appear is the AES time the second
%one os the Main time (NBV+AES+MaPP)
