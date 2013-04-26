%Edoardo Cacciavillani A97501678  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%MPLF %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function MainFieldD 
clear all
close all
clc

%Global variables %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global M MM U start goal km Mapp

% Map Data %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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

Initialize; %da rivedere!
PATH=ComputeShortestPath

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

counter=1; %da sostituire

while counter<1
    counter=counter+1;
    MappOld=Mapp;
    Mapp=MP{counter};
    Check=MappOld-Mapp;
    [r,c]=find(Check==1 | Check==-1);
    changed=[r,c];
    trigger=isempty(changed);
    if trigger ~= 1
        km=km+distance(start(1),start(2),PATH(2,1),PATH(2,2));
        start=PATH(2,:);
        [itx,~]=size(changed);
        for ii=1:itx
            for iii=0:1
                for jjj=0:1
                    UpdateVertex(changed(ii,1)+iii,changed(ii,2)+jjj);
                end
            end
            %cold=MM{changed(ii,1),changed(ii,2)};
            %[xcold,~]=size(cold); 
            %for ic=1:xcold
            %    cn(ic)=cost(changed(ii,1),changed(ii,2),cold(ic,1),cold(ic,2));
            %end
            %cnew=[cold(:,1:2),cn'];
            %MM{changed(ii,1),changed(ii,2)}=cnew;
            %cold=[];
            %cnew=[];
            %cn=[];
            %UpdateVertex(changed(ii,1),changed(ii,2))
        end
        PATH=ComputeShortestPath
    end
end
        
    