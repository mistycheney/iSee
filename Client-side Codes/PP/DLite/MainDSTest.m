function MainDS 
clear all
close all
clc
tic
%slast = sgoal ricorda questo nell'uso delle variabili
global M MM U start goal km Mapp%start � dinamico e coincide con la posizione iniziale, poi del robot

%for n=1:4 %Carico ognuna della 58 posizioni %
%
%MP{n}=load(int2str(n));
%end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

start=[4,6];
goal=[8,2];
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
0	0	0	0	0	0	0;
0	0	0	0	0	0	0;
0	0	0	0	0	0	0;
];
MP{3}=[0 0	0	0	0	0	0;
0	0	0	0	1	0	0;
0	0	1	0	1	0	0;
0	0	1	0	0	0	0;
0	0	1	1	1	0	0;
0	0	0	0	1	0	0;
0	0	0	0	0	0	0;
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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Initialize;
PATH=ComputePath(goal)
PATH(1,:)=[start(1),start(2)];
counter=1;


while counter<3 %PATH(counter)~=goal
    %manca una parte in cui si aggiorna la posizione del robot.
    counter=counter+1;
    trip=[];%???
    MappOld=Mapp;
    Mapp=MP{counter};
    Check=MappOld-Mapp;
    [r,c]=find(Check==1 | Check==-1);
    changed=[r,c]
    trigger=isempty(changed);
    if trigger~=1
        km=km+distance(PATH(2,1),PATH(2,2),start(1),start(2));
        start=[4,4]
        [ic,yc]=size(changed);
        for i=1:ic
            cold=MM{changed(i,1),changed(i,2)};
            [xcold,ycold]=size(cold); %controlla se pu� essere messo embedded
            for ii=1:xcold
                cn(ii)=cost(changed(i,1),changed(i,2),cold(ii,1),cold(ii,2));
            end
            cnew=[cold(:,1:2),cn'];
            cn=[];
            MM{changed(i,1),changed(i,2)}=cnew;
            for ii=1:xcold
                if cold(ii,3)>cnew(ii,3)
                    if changed(i,1)~=goal(1) || changed(i,2)~=goal(2)
                        iff=1
                        M{changed(i,1),changed(i,2)}(2)=min(M{changed(i,1),changed(i,2)}(2),cnew(ii,3)+M{cnew(ii,1),cnew(ii,2)}(1));
                    end
            elseif M{changed(i,1),changed(i,2)}(2)==cold(ii,3)+M{cnew(ii,1),cnew(ii,2)}(1)
                    if changed(i,1)~=goal(1) || changed(i,2)~=goal(2) 
                        rhschanged=M{changed(i,1),changed(i,2)}(2)
                        iff=2
                        M{changed(i,1),changed(i,2)}(2)=minsurrounding(changed(i,1),changed(i,2));
                        rhsnew=M{changed(i,1),changed(i,2)}(2)     %%molto dispendioso e non ottimizzato... pensa ad un'ottimizzazione
                    end
                end
               
            end
            cold=[]; 
            cnew=[];%FAI MOLTA ALTTENZIONE QUESTO UpVertex dentro o fuori dal for potrebbe cabiare MOLTO!!!!!!!!!!!!!!!!!!!!!!!!
        end
            for ii=1:ic
                UpVertex(changed(ii,1),changed(ii,2));
            end
    end
        PATH=ComputePath(goal)
        
        
end
