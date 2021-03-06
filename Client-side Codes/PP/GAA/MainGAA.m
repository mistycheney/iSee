%Edoardo Cacciavillani A97501678  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%MPLF %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% An implementation of GAA* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function MainGAA
clear all
close all
clc
tic
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

global count dh pcost M start goal OPEN MM Mapp

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

OPEN=[];
count=1;
dh(1)=0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
goal=[4,6];
start=[8,2];
MP{1}=[0	0	0	0	0	0	0;
0	0	0	0	1	0	0;
0	0	1	0	1	0	0;
0	0	1	0	0	0	0;
0	0	1	0	1	0	0;
0	0	0	0	1	0	0;
0	0	0	0	0	0	0;
0	0	0	0	0	0	0;
0	0	0	0	0	0	0;
];
Mapp=MP{1};
MP{2}=[0	0	0	0	0	0	0;
0	0	0	0	1	0	0;
0	0	1	0	1	0	0;
0	0	1	0	0	0	0;
0	0	1	0	1	0	0;
0	0	0	0	1	0	0;
0	0	0	0	1	0	0;
0	0	0	0	0	0	0;
0	0	0	0	0	0	0;
];
MP{3}=[0 0	0	0	0	0	0;
0	0	0	0	1	0	0;
0	0	1	0	1	0	0;
0	0	1	0	0	0	0;
0	0	1	1	1	0	0;
0	0	0	0	1	0	0;
0	0	1	0	0	0	0;
0	0	0	0	0	0	0;
0	0	0	0	0	0	0;
];
MP{4}=[0	0	0	0	0	0	0;
0	0	0	0	1	0	0;
0	1	1	0	1	0	0;
0	0	1	0	0	0	0;
0	0	1	1	1	0	0;
0	0	0	0	1	0	0;
0	1	0	0	0	0	0;
0	0	0	0	0	0	0;
0	0	0	0	0	0	0;
];
for i=1:10
    MP{4+i}=MP{4};
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

[msx,msy]=size(Mapp);
M=cell(msx,msy);
for i=1:msx
    for j=1:msy
        M{i,j}=[0,0,0,0,0,0]; %[gcost,hcost,isinOPEN,search,treeX,treeY]
        MM{i,j}=ar8(i,j);
    end
end

%for all s belonging to S search(s)=0 is assigne in the for cycle.

while start(1) ~= goal(1) || start(2) ~= goal(2)
    Initialize(start(1),start(2));
    Initialize(goal(1),goal(2));
    M{start(1),start(2)}(1)=0;
    OPEN=[OPEN;[M{start(1),start(2)}(1)+M{start(1),start(2)}(2),start(1),start(2)]];
    M{start(1),start(2)}(3)=1;
    PATH=ComputePath
    if isempty(OPEN)
        pcost(count)=Inf;
    else
        pcost(count)=M{goal(1),goal(2)}(1);
    end
    %here you can change start or goal%
    start=PATH(end-1,:)
    newgoal=goal;
    if goal(1) ~= newgoal(1) || goal(2) ~= newgoal(2)
        Initialize(newgoal(1),newgoal(2));
        if M{newgoal(1),newgoal(2)}(1)+M{newgoal(1),newgoal(2)}(2)<pcost(count)
            M{newgoal(1),newgoal(2)}(2)=pcost(count)-M{newgoal(1),newgoal(2)}(1);
        end
        dh(count+1)=dh(count)+M{newgoal(1),newgoal(2)}(2);
        goal=newgoal;
    else
        dh(count+1)=dh(count);
    end
    count=count+1;
    %CHANGE IN MAP COST
    MappOld=Mapp;
    Mapp=MP{count}; %check if counter
    Check=MappOld-Mapp;
    [sO,~]=size(OPEN);
    for O=sO:-1:1
        M{OPEN(O,2),OPEN(O,3)}(3)=0;
    end
    OPEN=[];
    [r1,c1]=find(Check==1); %questi sono i decreased
    [r2,c2]=find(Check==-1);
    changedD=[r1,c1];
    changedU=[r2,c2]; %needed??
    [sc1,~]=size(changedD);
    for i=1:sc1
        if changedD(i,1)==goal(1) && changed(i,2)==goal(2)
            changedD(i,:)=[];
        end
    end
    for i=1:sc1
        Initialize(changedD(i,1),changedD(i,2));
        list=MM{changedD(i,1),changedD(i,2)};
        [ix,~]=size(list);
        for ii=1:ix
            Initialize(list(ii,1),list(ii,2));
        end
        for ii=1:ix
            if M{changedD(i,1),changedD(i,2)}(2)>cost(changedD(i,1),changedD(i,2),list(ii,1),list(ii,2))+M{list(ii,1),list(ii,2)}(2)
            M{changedD(i,1),changedD(i,2)}(2)=cost(changedD(i,1),changedD(i,2),list(ii,1),list(ii,2))+M{list(ii,1),list(ii,2)}(2);
                if M{changedD(i,1),changedD(i,2)}(3)==1
                OPEN(OPEN(:,2)==changedD(i,1) & OPEN(:,3)==changedD(i,2))=[M{chagedD(i,1),changed(i,2)}(2),changedD(i,1),changedD(i,2)];
                else
                OPEN=[OPEN;[M{chagedD(i,1),changedD(i,2)}(2),changedD(i,1),changedD(i,2)]];
                M{changedD(i,1),changedD(i,2)}(3)=1;
                end
            end
        end
        list=[];
    end
    OPEN=sortrows(OPEN);
    while isempty(OPEN) ~= 1
        sx=OPEN(1,2)
        sy=OPEN(1,3)
        OPEN(1,:)=[];%non sicuro...
        M{sx,sy}(3)=0;
        list=MM{sx,sy};
        [ix,~]=size(list);
        for ii=1:ix
            Initialize(list(ii,1),list(ii,2));
            if M{list(ii,1),list(ii,2)}(2)>M{sx,sy}(2)+cost(list(ii,1),list(ii,2),list(ii,:),sx,sy);
                M{list(ii,1),list(ii,2)}(2)=M{sx,sy}(2)+cost(list(ii,1),list(ii,2),list(ii,:),sx,sy);
                if M{list(ii,1),list(ii,2)}(3)==1
                    OPEN(OPEN(:,2)==list(ii,1) & OPEN(:,3)==list(ii,2))=[M{list(ii,1),list(ii,2)}(2),list(ii,1),list(ii,2)];
                else
                    OPEN=[OPEN;[M{list(ii,1),list(ii,2)}(2),list(ii,1),list(ii,2)]];
                    M{list(ii,1),list(ii,2)}(3)=1;
                end
            end
        end
    end
end
                
            toc      
        
            
        
        
        
       