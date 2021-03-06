%Edoardo Cacciavillani A97501678 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%MPLF %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [PATH] = ComputeShortestPath

%Global variables %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global M MM U start goal

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

while (U(1,1:2)-CKey(start(1),start(2))<=0) | (M{start(1),start(2)}(2)>M{start(1),start(2)}(1)) 
    kold=U(1,1:2);
    x=U(1,3);   %U.Pop
    y=U(1,4);   %U.Pop
    U(1,:)=[];  %U.Pop
    KEY=CKey(x,y);
    a=KEY(1);
    b=KEY(2);
    list=[];
    if (kold(1)<a) || (kold(1)==a && kold(2)<b)
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
counter=1;
PATH(counter,1:2)=[start(1),start(2)];
while (PATH(counter,1)~=goal(1) || PATH(counter,2)~=goal(2)) && (counter<10) 
    list=MM{PATH(counter,1),PATH(counter,2)};
    [idx,~]=size(list);
    for i=1:idx
        costi(i,:)=[M{list(i,1),list(i,2)}(1)+cost(PATH(counter,1),PATH(counter,2),list(i,1),list(i,2)),list(i,1),list(i,2)];
    end
    counter=counter+1;
    costf=sortrows(costi);
    if costf(1,2)~=PATH(counter-1,1) || costf(1,3)~=PATH(counter-1,2) 
    PATH(counter,:)=costf(1,2:3);
    else
    PATH(counter,:)=costf(2,2:3);
    end
end

