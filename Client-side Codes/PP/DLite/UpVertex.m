function UpVertex(x,y)

global M U

if M{x,y}(1)~=M{x,y}(2) && M{x,y}(3)==1
    U(U(:,3)==x & U(:,4)==y,1:2)=CKey(x,y);
elseif M{x,y}(1)~=M{x,y}(2) && M{x,y}(3)==0
    U=[U;[CKey(x,y),x,y]];
    M{x,y}(3)=1;
elseif M{x,y}(1)==M{x,y}(2) && M{x,y}(3)==1
    U(U(:,3)==x & U(:,4)==y,:)=[];
    M{x,y}(3)=0;
end

U=sortrows(U);