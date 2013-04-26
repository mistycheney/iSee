function demoProgram()
addpath('../data');
%% load data
load testData2;
% load testData3;
%% initialize SLAM
% laserType % 1 for lms 100, 0 for HokuyoUrg04Lx
% mapRes    = 0.05; % meter
% mapWidth  = 60;   % meter
cols      = 1 + ceil(mapWidth/mapRes);
rows      = 1 + ceil(mapWidth/mapRes);
map       = 0.5*ones(rows,cols);
%%
figure(2);
set(gcf,'color',[1 1 1]);
%%
laserSlamMex('Initialization',laserType,mapRes,mapWidth);
%% process each frame
frameNum = size(laser,2);
slamTime = 0;
for i=1:frameNum
    %% 
    tic
    laserSlamMex('Slam',laser(:,i)');
    slamTime = slamTime+toc;
   %% get map
if mod(i,10)==0   
   tic
   
    [pose poseGrid] = laserSlamMex('getMapAndPose',map);
   
   toc;
  %% display
  cla
  imshow(map);
  hold on;
  plot(poseGrid(2),poseGrid(1),'or','MarkerSize',5,'MarkerFaceColor',[1 0 0]);
  drawnow;
  i
  slamTime/i
end
end
%% release memory
 laserSlamMex('releaseMemory');
