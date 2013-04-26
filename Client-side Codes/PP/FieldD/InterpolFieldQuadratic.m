%Edoardo Cacciavillani A97501678  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%MPLF %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [pathXY]=InterpolFieldQuadratic(config) %You can Add xp for big cells, otherwise it's default on 0.5

% Data %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

global MapCost Mapp start goal Mov NC

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

xp=0.5;
XG=goal(2)-(start(2)+1);
YG=-goal(1)+(start(1)+1); 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

[Aij,A,B,C,D,M,pathXY]=InitializationGrid(start,config,xp);
[ABCD1,ABCD2,ABCD3,M1,M2,M3]=GridMoving(Aij,config);
     
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%dX=(B-A)+y*(D+A-B-C)+M*((x-xp)^2+(y-yp)^2)^-0.5*(x-xp);
%dY=(C-A)+x*(D+A-B-C)+M*((x-xp)^2+(y-yp)^2)^-0.5*(y-yp);
%fXY=A*(1-x)*(1-y)+B*(1-y)*x+C*(1-x)*y+D*x*y+M*((x-xp)^2+(y-yp)^2)^0.5;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

i=1;
while (pathXY(i,1)-XG>=0.5 || pathXY(i,2)-YG>=0.5) 
%quadratic parameters
a1=2*A-2*C+4*M;
b1=C-A-4*M; 
c1=C;    
a2=2*D-2*C+4*M;
b2=C-D-4*M;
c2=C;
a3=2*B-2*D+4*M;
b3=D-B-4*M;
c3=D;
%x=0 y variabile
f0Y=@(y)a1*y^2+b1*y+c1+M*((xp)^2+y^2)^0.5+M1;
%y=0 x variabile
fX1=@(x)a2*x^2+b2*x+c2+M*((x-xp)^2+1)^0.5+M2;
%x=1 t variabile
f1Y=@(y)a3*y^2+b3*y+c3+M*((1-xp)^2+y^2)^0.5+M3;

[Yx0,fYx0]=fminbnd(f0Y,0,1);
[Xy1,fXy1]=fminbnd(fX1,0,1);
[Yx1,fYx1]=fminbnd(f1Y,0,1);
[~,mov]=min([fYx0,fXy1,fYx1]);

if mov==1
    np=[-xp,Yx0];
    Aij=ABCD1(1,:);
    Bij=ABCD1(2,:);
    Cij=ABCD1(3,:);
    Dij=ABCD1(4,:);
    M=M1;
elseif mov==2
    np=[Xy1-xp,1];
    Aij=ABCD2(1,:);
    Bij=ABCD2(2,:);
    Cij=ABCD2(3,:);
    Dij=ABCD2(4,:);
    M=M2;
else
    np=[(1-xp),Yx1];
    Aij=ABCD3(1,:);
    Bij=ABCD3(2,:);
    Cij=ABCD3(3,:);
    Dij=ABCD3(4,:);
    M=M3;
end
A=MapCost(Aij(1),Aij(2));
B=MapCost(Bij(1),Bij(2));
C=MapCost(Cij(1),Cij(2));
D=MapCost(Dij(1),Dij(2));

%Update position in cartesian grid %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if config(2)==1
    pathXY(i+1,:)=[pathXY(i,1)+np(2),pathXY(i,2)-np(1)];
elseif config(2)==2
    pathXY(i+1,:)=[pathXY(i,1)+np(1),pathXY(i,2)+np(2)];
elseif config(2)==3
    pathXY(i+1,:)=[pathXY(i,1)-np(2),pathXY(i,2)+np(1)];
else
    pathXY(i+1,:)=[pathXY(i,1)-np(1),pathXY(i,2)-np(2)];
end

%Relative position in cell reference %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

config=NC{config(1),config(2)}(:,mov)';
yA=-Aij(1)+start(1)+1; 
xA=Aij(2)-(start(2)+1);  
RelP=[abs(xA-pathXY(i+1,1)),abs(yA-pathXY(i+1,2))]; 
if config(2)==1
    xp=RelP(2);
elseif config(2)==2
    xp=RelP(1);
elseif config(2)==3
    xp=RelP(2);
else
    xp=RelP(1);
end

%Calculate Nexts %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

[ABCD1,ABCD2,ABCD3,M1,M2,M3]=GridMoving(Aij,config);


i=i+1;
end