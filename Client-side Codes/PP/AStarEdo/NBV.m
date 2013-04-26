function [xG,yG,found]=NBV(BEL,pos)

%%MapperNBV%%
%%A Program By Edoardo Cacciavillani%%
%%ILTP%%
%%A97501678%%

NBV=zeros(120,120);
FR=zeros(120,120);
MapN=zeros(120,120);

%NBV Greedy %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Map Cleaning %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for i=1:120
    for j=1:120
        if BEL(i,j)>0.95
            MapN(i,j)=1;
        elseif BEL(i,j)<=0.95 && BEL(i,j)>0.05
            MapN(i,j)=0.5;
        else
            MapN(i,j)=0;
        end
    end
end

%NBV Greedy %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Map weightneing %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%cell has to be reachable, than if MapN()==0 we add a very low value, while
%if it is at the border with an unknown area we add a positive value. If it
%is surrounded by free cell we subrtact a small value. This part can be
%upgraded with more sophisticated weightening methods that consider other
%paramenters.

for i=4:116
    for j=4:116
        for ii=-3:3
            for jj=-3:3
                if MapN(i+ii,j+jj)==0.5
                    NBV(i,j)=NBV(i,j)+20;
                elseif MapN(i+ii,j+jj)==1
                    NBV(i,j)=NBV(i,j)-10;
                elseif MapN(i+ii,j+jj)==0
                    NBV(i,j)=NBV(i,j)-10000;
                end
            end
        end
    end
end

%to be acceptable the total value should be between 0 and 400, this means
%that there are enough free cells in the proximity to consider the cell
%reachable and that the cell is at the border between known and unknown
%area. 

C=1;
for i=4:116
    for j=4:116
        if NBV(i,j)>-50 && NBV(i,j)<300
            pgoal(C,:)=[i,j,distance(i,j,pos(1),pos(2))];
            C=C+1;
        end
    end
end

pgoal=sortrows(pgoal,3); %Sort of the rows based on distance
found=0;

while found==0
for ii=-3:3
    for jj=-3:3
        for i=-3:3
            for j=-3:3
                if found ==0
                if MapN(pgoal(1,1)+ii+i,pgoal(1,2)+j+jj)==1
                    FR(pgoal(1,1)+ii,pgoal(1,2)+jj)=FR(pgoal(1,1)+ii,pgoal(1,2)+jj)+1;
                else
                    FR(pgoal(1,1)+ii,pgoal(1,2)+jj)=FR(pgoal(1,1)+ii,pgoal(1,2)+jj)-100;
                end
                end
            end
        end
            if FR(pgoal(1,1)+ii,pgoal(1,2)+jj)>0
                xG=pgoal(1,1)+ii;
                yG=pgoal(1,2)+jj;
                found=1;
            end
    end
end
if found ~=1
    pgoal(1,:)=[];
end
end





