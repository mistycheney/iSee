%Edoardo Cacciavillani A97501678  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%MPLF %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% An implementation of GAA* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function Initialize(x,y)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

global count dh pcost M goal 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if M{x,y}(4)~= count && M{x,y}(4)~=0
    if M{x,y}(1)+M{x,y}(2)<pcost(M{x,y}(4))
        M{x,y}(2)=pcost(M{x,y}(4))-M{x,y}(1);
    end
    M{x,y}(2)=M{x,y}(2)-(dh(count)-dh(M{x,y}(4)));
    M{x,y}(2)=max(M{x,y}(2),distance(x,y,goal(1),goal(2)));
    M{x,y}(1)=Inf;
elseif M{x,y}(4)==0
    M{x,y}(1)=Inf;
    M{x,y}(2)=distance(x,y,goal(1),goal(2));
end
M{x,y}(4)=count;

