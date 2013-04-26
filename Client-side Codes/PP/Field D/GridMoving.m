function[ABCD1,ABCD2,ABCD3,M1,M2,M3]=GridMoving(Aij,config)

global Mov Mapp

[mx,my]=size(Mapp);

CNT=Mov{config(1),config(2)};
CNtile1=[CNT(:,(1*2)-1),CNT(:,(1*2))];
CNtile2=[CNT(:,(2*2)-1),CNT(:,(2*2))];
CNtile3=[CNT(:,(3*2)-1),CNT(:,(3*2))];
ABCD1=[Aij+CNtile1(1,:); Aij+CNtile1(2,:); Aij+CNtile1(3,:); Aij+CNtile1(4,:);];
ABCD2=[Aij+CNtile2(1,:); Aij+CNtile2(2,:); Aij+CNtile2(3,:); Aij+CNtile2(4,:);];
ABCD3=[Aij+CNtile3(1,:); Aij+CNtile3(2,:); Aij+CNtile3(3,:); Aij+CNtile3(4,:);];
pick1=sortrows(ABCD1);
pick2=sortrows(ABCD2);
pick3=sortrows(ABCD3);

if pick1(1,1)<=0 || pick1(1,1)>mx || pick1(1,2)<=0 || pick1(1,2)>my
    M1=Inf;
elseif Mapp(pick1(1,1),pick1(1,2))==0
    M1=10;
else
    M1=Inf;
end

if pick2(1,1)<=0 || pick2(1,1)>mx || pick2(1,2)<=0 || pick2(1,2)>my
    M2=Inf;
elseif Mapp(pick2(1,1),pick2(1,2))==0
    M2=10;
else
    M2=Inf;
end

if pick3(1,1)<=0 || pick3(1,1)>mx || pick3(1,2)<=0 || pick3(1,2)>my
    M3=Inf;
elseif Mapp(pick3(1,1),pick3(1,2))==0
    M3=10;
else
    M3=Inf;
end