function v=ComputeCost(x,y,i,j,ii,jj)

global M Mapp 
[limx,limy]=size(Mapp);

if i ~= x && y ~= j
    s1=[ii,jj];
    s2=[i,j];
else
    s1=[i,j];
    s2=[ii,jj];
end

if s2(1)==x-1 && s2(2)==y+1
    c=[x-1,j];
    corner=1;
elseif s2(1)==x-1 && s2(2)==y-1
    c=[x-1,j-1];
    corner=2;
elseif s2(1)==x+1 && s2(2)==y-1
    c=[x,j-1];
    corner=3;
elseif s2(1)==x+1 && s2(2)==y+1
    c=[x,j];
    corner=4;
end

if corner==1 
    if x==s1(1)
        b=[x,y];
    else
        b=[x-1,y-1];
    end
elseif corner==2
    if y==s1(2)
        b=[x-1,y];
    else
        b=[x,y-1];
    end
elseif corner==3
    if x==s1(1)
        b=[x-1,y-1];
    else
        b=[x,y];
    end
elseif corner==4
    if y==s1(2)
        b=[x,y-1];
    else
        b=[x-1,y];
    end
end

if c(1)==0 || c(2)==0 || c(1)==limx+1 || c(2)==limy+1
    ccost=Inf;
else
    if Mapp(c(1),c(2))==0
    ccost=10;
    else
    ccost=Inf;
    end
end
if b(1)==0 || b(2)==0 || b(1)==limx+1 || b(2)==limy+1
    bcost=Inf;
else
    if Mapp(b(1),b(2))==0
    bcost=10;
    else
    bcost=Inf;
    end
end

if min(ccost,bcost)==Inf
    v=Inf;
elseif M{s1(1),s1(2)}(1)<=M{s2(1),s2(2)}(2)
    v=min(ccost,bcost)+M{s1(1),s1(2)}(1);
else
    f=M{s1(1),s1(2)}(1)-M{s2(1),s2(2)}(2);
    if f<=bcost
        if ccost<=f
            v=ccost*1.4142+M{s2(1),s2(2)}(1);%% NON MI PIACE IL FLOATING!!!
        else
            y=min(f/sqrt(ccost^2-f^2),1);
            v=ccost*sqrt(1+y^2)+f*(1-y)+M{s2(1),s2(2)}(1);
        end
    else
        if ccost<=bcost
            v=ccost*1.4142+M{s2(1),s2(2)}(1);
        else
            x=1-min(bcost/sqrt(ccost^2-bcost^2),1);
            v=ccost*sqrt(1+(1-x)^2)+bcost*x+M{s2(1),s2(2)}(1);
        end
    end
end

