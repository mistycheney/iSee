%Edoardo Cacciavillani A97501678  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%MPLF %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function UpdateVertex(x,y)

global M U goal

if x~=goal(1) || y~=goal(2)
    M{x,y}(2)=minsurrounding(x,y);
end
if M{x,y}(3)==1
    U(U(:,3)==x & U(:,4)==y,:)=[];
    M{x,y}(3)=0;
end
if M{x,y}(1)~=M{x,y}(2)
    Key=CKey(x,y);
    U=[U;[Key,x,y]];
    M{x,y}(3)=1; 
end

U=sortrows(U);

    