%Edoardo Cacciavillani A97501678  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%MPLF %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% An implementation of GAA* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [PATH] = ComputePath

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

global M goal OPEN MM start

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

while M{goal(1),goal(2)}(1)>OPEN(1,1)
    x=OPEN(1,2);
    y=OPEN(1,3);
    OPEN(1,:)=[];
    M{x,y}(3)=0;
    list=MM{x,y};
    [ix,~]=size(list);
    for i=1:ix
        Initialize(list(i,1),list(i,2));
        if M{list(i,1),list(i,2)}(1)>M{x,y}(1)+cost(x,y,list(i,1),list(i,2));
            M{list(i,1),list(i,2)}(1)=M{x,y}(1)+cost(x,y,list(i,1),list(i,2));
            M{list(i,1),list(i,2)}(5:6)=[x,y]; %tree(succ(s,a)) 
            if M{list(i,1),list(i,2)}(3)==1
                OPEN(OPEN(:,2)==list(i,1) & OPEN(:,3)==list(i,2),:)=[M{list(i,1),list(i,2)}(2)+M{list(i,1),list(i,2)}(1),list(i,1),list(i,2)];
            else
                OPEN=[OPEN;[M{list(i,1),list(i,2)}(2)+M{list(i,1),list(i,2)}(1),list(i,1),list(i,2)]];
                M{list(i,1),list(i,2)}(3)=1;
            end
        end
    end
    OPEN=sortrows(OPEN);
end
c=1;
PATH(c,:)=[goal(1),goal(2)];
while PATH(c,1)~=start(1) || PATH(c,2)~=start(2)
    PATH(c+1,:)=M{PATH(c,1),PATH(c,2)}(5:6);
    c=c+1;
end
    
%qui ci sta il design path!!

            
    