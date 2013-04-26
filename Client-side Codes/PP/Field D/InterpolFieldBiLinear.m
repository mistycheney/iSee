%Edoardo Cacciavillani A97501678  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%MPLF %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all
close all
clc

% Data %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
xp=0.8;
yp=0;
XG=-4;
YG=-2;
Map=[0	0	0	0	0	0	0;
0	0	0	0	1	0	0;
0	0	1	0	1	0	0;
0	0	1	0	0	0	0;
0	0	1	0	1	0	0;
0	0	0	0	1	0	0;
0	0	0	0	0	0	0;
0	0	0	0	0	0	0;
0	0	0	0	0	0	0;
];
Aij=[5,6];
pathXY(1,:)=[0 0.8];
config=[2,3];
Mov=cell(2,4);
NC=cell(2,4);
Mov{1,2}=[0 0 -1 0 -1 +1;
        -1 0 -1 +1 0 +1;
        0 -1 -2 0 -1 2;
        -1 -1 -2 1 0 2;];
Mov{1,4}=-Mov{1,2};
Mov{2,1}=[0 0 0 1 1 1;
        0 1 1 1 1 0;
        -1 0 0 2 2 1;
        -1 1 1 2 2 0];
Mov{2,3}=-Mov{2,1};
NC{1,2}=[2 1 2;
         3 2 1];
NC{1,4}=[2 1 2;
         1 4 3];
NC{2,1}=[1 2 1;
         2 1 4];
NC{2,3}=[1 2 1;
         4 3 2];
MM=[Inf	Inf	Inf	Inf	Inf	Inf	Inf	Inf;
Inf	Inf	Inf	Inf	Inf	Inf	Inf	Inf;
52.05540501078997	50.0	52.05540501078997	55.295230781918576	Inf	Inf	Inf	Inf;
42.33083863091231	40.0	42.33083863091231	45.98794461250131	55.98794461250131	60.164869618142305	Inf	Inf;
32.70310727512816	30.0	32.70310727512816	36.921010687132664	46.921010687132664	51.06301068713266	Inf	Inf;
23.243858931009644	20.0	23.243858931009644	28.28413562373095	36.921010687132664	45.98794461250131	55.98794461250131	Inf;
14.142	10.0	14.142	23.243858931009644	32.70310727512816	42.33083863091231	52.05540501078997	Inf;
10.0	0.0	10.0	20.0	30.0	40.0	50.0	Inf;
14.142	10.0	14.142	23.243858931009644	32.70310727512816	42.33083863091231	52.05540501078997	Inf;
23.243858931009644	20.0	23.243858931009644	28.28413562373095	36.921010687132664	45.98794461250131	Inf	Inf
];
A=MM(5,6);
B=MM(4,6);
C=MM(5,5);
D=MM(4,5);
M=10;
M1=Inf;
M2=10;
M3=Inf;
% Per la mappa usa il sortrows! Ma si potrebbe fare una cella anche li!     
     
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%dX=(B-A)+y*(D+A-B-C)+M*((x-xp)^2+(y-yp)^2)^-0.5*(x-xp);
%dY=(C-A)+x*(D+A-B-C)+M*((x-xp)^2+(y-yp)^2)^-0.5*(y-yp);
%fXY=A*(1-x)*(1-y)+B*(1-y)*x+C*(1-x)*y+D*x*y+M*((x-xp)^2+(y-yp)^2)^0.5;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
i=1;
while (pathXY(i,1)-XG>=0.5 || pathXY(i,2)-YG>=0.5) %&& i<6

%x=0 y variabile
f0Y=@(y)A*(1-y)+C*y+M*((xp)^2+y^2)^0.5+M1;
%y=0 x variabile
fX1=@(x)C*(1-x)+D*x+M*((x-xp)^2+1)^0.5+M2;
%x=1 t variabile
f1Y=@(y)B*(1-y)+D*y+M*((1-xp)^2+y^2)^0.5+M3;

[Yx0,fYx0]=fminbnd(f0Y,0,1);
[Xy1,fXy1]=fminbnd(fX1,0,1);
[Yx1,fYx1]=fminbnd(f1Y,0,1);


a=[fYx0,fXy1,fYx1];
[val,mov]=min(a);

if mov==1
    mov=mov
    np=[-xp,Yx0] %credo ci sia qualcosa che non va!
elseif mov==2
     mov=mov
     np=[Xy1-xp,1]
else 
     mov=mov
     np=[(1-xp),Yx1]
end

% Aggiorna posizione in coordinate cartesiane %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if config(2)==1
    con=1
    pathXY(i+1,:)=[pathXY(i,1)+np(2),pathXY(i,2)-np(1)]%da rivedere
elseif config(2)==2
    con=2
    pathXY(i+1,:)=[pathXY(i,1)+np(1),pathXY(i,2)+np(2)]%da rivedere
elseif config(2)==3
    con=3
    pathXY(i+1,:)=[pathXY(i,1)-np(2),pathXY(i,2)+np(1)]%da rivedere
else
    con=4
    pathXY(i+1,:)=[pathXY(i,1)-np(1),pathXY(i,2)-np(2)]%da rivedere
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Evolvi posizione e sistema %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
CT=Mov{config(1),config(2)};
Ctile=[CT(:,(mov*2)-1),CT(:,(mov*2))];
Aijn=Aij+Ctile(1,:);
A=MM(Aijn(1),Aijn(2));
Bij=Aij+Ctile(2,:);
B=MM(Bij(1),Bij(2));
Cij=Aij+Ctile(3,:);
C=MM(Cij(1),Cij(2));
Dij=Aij+Ctile(4,:);
D=MM(Dij(1),Dij(2));
Aij=Aijn;

ABCD=[Aij;Bij;Cij;Dij];
pick=sortrows(ABCD)

if Map(pick(1,1),pick(1,2))==0
    M=10
else
    M=Inf
end
config=NC{config(1),config(2)}(:,mov)'
yA=-Aij(1)+5;
xA=Aij(2)-6; 
RelP=[abs(xA-pathXY(i+1,1)),abs(yA-pathXY(i+1,2))] 
if config(2)==1
    xp=RelP(2)
elseif config(2)==2
    xp=RelP(1)
elseif config(2)==3
    xp=RelP(2)
else
    xp=RelP(1)
end

%calcola costo successiva %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
CNT=Mov{config(1),config(2)};
CNtile1=[CNT(:,(1*2)-1),CNT(:,(1*2))];
CNtile2=[CNT(:,(2*2)-1),CNT(:,(2*2))];
CNtile3=[CNT(:,(3*2)-1),CNT(:,(3*2))];
Aij1=Aij+CNtile1(1,:);
Bij1=Aij+CNtile1(2,:);
Cij1=Aij+CNtile1(3,:);
Dij1=Aij+CNtile1(4,:);
Aij2=Aij+CNtile2(1,:);
Bij2=Aij+CNtile2(2,:);
Cij2=Aij+CNtile2(3,:);
Dij2=Aij+CNtile2(4,:);
Aij3=Aij+CNtile3(1,:);
Bij3=Aij+CNtile3(2,:);
Cij3=Aij+CNtile3(3,:);
Dij3=Aij+CNtile3(4,:);
ABCD1=[Aij1;Bij1;Cij1;Dij1]
ABCD2=[Aij2;Bij2;Cij2;Dij2]
ABCD3=[Aij3;Bij3;Cij3;Dij3]
pick1=sortrows(ABCD1);
pick2=sortrows(ABCD2);
pick3=sortrows(ABCD3);
if Map(pick1(1,1),pick1(1,2))==0
    M1=10
else
    M1=Inf
end
if Map(pick2(1,1),pick2(1,2))==0
    M2=10
else
    M2=Inf
end
if Map(pick3(1,1),pick3(1,2))==0
    M3=10
else
    M3=Inf
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



i=i+1
pathXY=pathXY;
end







