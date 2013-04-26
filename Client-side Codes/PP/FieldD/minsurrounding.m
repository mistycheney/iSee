function minimum=minsurrounding(i,j)
global ME 
list=ME{i,j};
[ixX,~]=size(list);
coost=zeros(ixX,1);
for ii=1:ixX
    coost(ii)=ComputeCost(i,j,list(ii,1),list(ii,2),list(ii,3),list(ii,4));
end
minimum=min(coost);
    