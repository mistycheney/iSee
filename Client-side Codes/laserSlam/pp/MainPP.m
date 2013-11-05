%Main Planner %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%A program by Edoardo Cacciavillani %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%MPLF %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all
close all
clc
tic
load testData3;
laserType =0;
mapRes    = 0.05; % meter
mapWidth  = 60;   % meter
cols      = 1 + ceil(mapWidth/mapRes);
rows      = 1 + ceil(mapWidth/mapRes);
map       = 0.5*ones(rows,cols);
laserSlamMex('Initialization',laserType,mapRes,mapWidth);
%Data input %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%every time we get new data...
for i=1:201
laserSlamMex('Slam',laser(:,i)');
[pose poseGrid] = laserSlamMex('getMapAndPose',map); %we have only the current pose
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

[goalx,goaly,found,pgoal]=NextBestView(map,poseGrid);
emptypgoal=0;
if found==1 % start path search only if we have a goal
goal=[goalx,goaly];
Mapp=MaPP(map,cols,rows);
[done,PATH]=AES(Mapp,poseGrid,goal);
    while done==0 && emptypgoal==0
        pgoal(1,:)=[];
        goal=pgoal(1,1:2);
        [done,PATH]=AES(Mapp,poseGrid,goal);
        emptypgoal=isempty(pgoal);
    end
else
    PATH=0;
end
PATH=PATH
toc

%NOTE in terms of time the first one to appear is the AES time the second
%one os the Main time (NBV+AES+MaPP)
%% release memory
 laserSlamMex('releaseMemory');