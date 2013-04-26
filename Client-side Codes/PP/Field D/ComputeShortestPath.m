%Edoardo Cacciavillani A97501678 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%MPLF %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [PATHCart] = ComputeShortestPath

%Global variables %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global M MM U start goal MapCost

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

while (U(1,1:2)-CKey(start(1),start(2))<=0) | (M{start(1),start(2)}(2)>M{start(1),start(2)}(1)) 
    kold=U(1,1:2);
    x=U(1,3);   %U.Pop
    y=U(1,4);   %U.Pop
    U(1,:)=[];  %U.Pop
    KEY=CKey(x,y);
    if (kold(1)<KEY(1)) || (kold(1)==KEY(1) && kold(2)<KEY(2))
        U=[U;[a,b,x,y]];
        M{x,y}(3)=1;
        U=sortrows(U);
    elseif M{x,y}(1)>M{x,y}(2)
        M{x,y}(1)=M{x,y}(2);
        list=MM{x,y};
        [idx,~]=size(list);
        for i=1:idx
            UpdateVertex(list(i,1),list(i,2));
        end
    else
        M{x,y}(1)=Inf;
        list=MM{x,y};
        list=[list;x,y,0];
        [idx,~]=size(list);
        for i=1:idx
            UpdateVertex(list(i,1),list(i,2));
        end
    end
end

%Calculate Path %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%Produce Map Cost %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[sx,sy]=size(M);
MapCost=Inf(sx,sy);
for i=1:sx
    for j=1:sy
        MapCost(i,j)=M{i,j}(1);
    end
end
PATHCart=InterpolFieldQuadratic([2,3]) %qui config bisogna vedere come funziona.

%THE END %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


