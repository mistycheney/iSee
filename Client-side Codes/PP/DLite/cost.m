function [co]=cost(x,y,i,j)
global Mapp

%sembra che ad un certo punto dia a tutti lo stesso valore... 10.

if Mapp(x,y)==1 || Mapp(i,j)==1
   co=inf;
elseif i==x || y==j 
   co=10;
elseif i==x-1 && j==y-1 && Mapp(x,y-1)==0 && Mapp(x-1,y)==0
   co=14;
elseif i==x-1 && j==y+1 && Mapp(x,y+1)==0 && Mapp(x-1,y)==0
   co=14;
elseif i==x+1 && j==y-1 && Mapp(x,y-1)==0 && Mapp(x+1,y)==0
   co=14;
elseif i==x+1 && j==y+1 && Mapp(x,y+1)==0 && Mapp(x+1,y)==0
   co=14;
else
   co=inf;
end
    

    
    
