function [done,PATH]=AES(Mapp,pos,goal)
%%AE*Algorithm %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%A path planning algorithm %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%Data %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

tic

[msizex,msizey]=size(Mapp);
OPCL=zeros(msizex,msizey);
M=cell(msizex,msizey);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%Map cleaning %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Not really needed in the this part could be done in main %%%%%%%%%%%%%%%%
k=1;
for i=1:msizex
    for j=1:msizey
        if Mapp(i,j) == 1 %1 per occupato
            OPCL(i,j)=-1;
            k=k+1;
        end
    end
end
CCount=k;

%AE* algorithm %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Initialize %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
doable=1;
done=0;
xP=pos(1);
xPa=xP;
yP=pos(2);
yPa=yP;
xG=goal(1);
yG=goal(2);
OCount=1;
hcost=distance(xP,yP,xG,yG);
M{xP,yP}=[1,xPa,yPa,0,0,hcost]; %cell vector [openclosed,parentx,parenty,gcost,hcost,fcost]
F(1,:)=[hcost,xP,yP];

%AE* algorithm %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Program %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if Mapp(xPa,yPa)==1 %check if inital position is ok
    doable=0;
    PATH=0;
    return
end

while (done~=1 && doable~=0) %condition to continue the search: path not found but foundable
    M{xPa,yPa}(1)=1;
    for i=-3:3               %check if the space the size of the robot is free
        for j=-3:3
            walkable=0;
            if Mapp(xPa+i,yPa+j) == 1
               M{xPa+i,yPa+j}(1)=-1;
               CCount=CCount+1;
               walkable=-1;
            end
            if walkable ==-1 %if spece is not free cell goes on closedlist
                M{xPa,yPa}(1)=-1;  
                CCount=CCount+1;
            end
        end
    end
    
%AE* algorithm %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
%Searching part - check sorrounding cells %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%should chenge the addedgcost part... we should not consider the central
%cell...
if  M{xPa,yPa}(1)~=-1
    for i=-1:1
        for j=-1:1
            if OPCL(xPa+i,yPa+j)==-1 %closed, don't do anything
            end
            if OPCL(xPa+i,yPa+j)==0  %unchecked, check and put on open
                hcost=distance(xPa+i,yPa+j,xG,yG);
                if i~=0 && j~=0
                    addedgcost=14;
                else
                    addedgcost=10;
                end
                gcost=M{xPa,yPa}(4)+addedgcost;
                fcost=gcost+hcost;
                M{xPa+i,yPa+j}=[1,xPa,yPa,gcost,fcost,hcost];
                OPCL(xPa+i,yPa+j)=1;
                OCount=OCount+1;
                F(OCount,:)=[fcost,xPa+i,yPa+j];
            elseif OPCL(xPa+i,yPa+j)==1 %open, verify parent 
                if i~=0 && j~=0
                    addedgcost=14;
                else
                    addedgcost=10;
                end
                gcost=M{xPa,yPa}(4)+addedgcost;
                if gcost<M{xPa+i,yPa+j}(4)
                    fcost=gcost+M{xPa+i,yPa+j}(6);
                    M{xPa+i,yPa+j}(4)=gcost;
                    M{xPa+i,yPa+j}(2)=xPa;
                    M{xPa+i,yPa+j}(3)=yPa;
                    M{xPa+i,yPa+j}(5)=fcost;
                    OCount=OCount+1;
                    F(OCount,:)=[fcost,xPa+i,yPa+j];
                end
            end
            if xPa+i==xG && yPa+j==yG %check if goal is reached
               M{xPa+i,yPa+j}=[1,xPa,yPa,0,0,0];
               done=1;    
            end
        end
    end
end
    good=0;
    F=sortrows(F,1);
    while good~=1 %check next cell to analize, choose lowest f value
    M{xPa,yPa}(1)=-1;
    OPCL(xPa,yPa)=-1;
    F(1,:)=[];
    OCount=OCount-1;
    xPa=F(1,2);
    yPa=F(1,3);
    if xPa>4 && xPa<msizex-4 && yPa>4 && yPa<msizey-4 %discard cell to close to border
        good=1;
    end
    end
    
    if CCount==(msizex*msizey)-1 %if every cell is closed there's no path
       doable=0;
    end
end

%AE* Algorithm %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Path builder %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

xPa=xG;
yPa=yG;
c=2;  
PATH(1,:)=[xG,yG];
did=0;
if done==1
    while did~=1
        PATH(c,:)=[M{xPa,yPa}(2) M{xPa,yPa}(3)];
        xPa=PATH(c,1);
        yPa=PATH(c,2);
        c=c+1;
        if (xPa==xP && yPa==yP) %ends when reach the initial position
            did=1;
        end
    end
end
if doable==0 %no path, path is null
    PATH=0;
end


[a,b]=size(PATH);
for t=1:a
    x=PATH(t,1);
    y=PATH(t,2);
    Mapp(x,y)=0.3;
end
toc

image(Mapp*100)                         
grid on
grid minor
