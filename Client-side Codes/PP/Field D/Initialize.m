function  Initialize

global U M MM ME start km goal Mapp

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

msize=size(Mapp);
km=0;
M=cell(msize(1)+1,msize(2)+1);

for i=1:msize(1)+1
    for j=1:msize(2)+1
M{i,j}=[inf, inf, 0]; %da rivedere perchè i valori son più di tre!!
    end
end

M{goal(1),goal(2)}(2)=0;
M{goal(1),goal(2)}(3)=1;

for i=1:msize(1)+1
    for j=1:msize(2)+1
        MM{i,j}=ar8(i,j);
        ME{i,j}=ed8(i,j);
    end
end


                
U=[distance(start(1),start(2),goal(1),goal(2)),0,goal(1),goal(2)];

