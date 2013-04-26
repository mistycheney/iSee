function [AR]=ar8(i,j)
global M
[idx,idy]=size(M);
AR=[i-1,j-1;
    i-1,j;
    i-1,j+1;
    i,j-1;
    i,j+1;
    i+1,j-1;
    i+1,j;
    i+1,j+1];

for i=8:-1:1
        if AR(i,1)==0 || AR(i,2)==0 || AR(i,1)==idx+1 || AR(i,2)==idy+1
            AR(i,:)=[];
        end
end