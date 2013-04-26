function minimum=minsurrounding(i,j)
global MM M 
list=MM{i,j};
[ixX,~]=size(list);
coost=[ixX,1];
for ii=1:ixX
    coost(ii)=cost(i,j,list(ii,1),list(ii,2))+M{list(ii,1),list(ii,2)}(1);
end
minimum=min(coost);
    