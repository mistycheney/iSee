function  Initialize

global U M MM start km goal Mapp
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
msize=size(Mapp);
km=0;
M=cell(msize(1),msize(2));
for i=1:msize(1)
    for j=1:msize(2)
M{i,j}=[inf, inf, 0];
    end
end
M{goal(1),goal(2)}(2)=0;
M{goal(1),goal(2)}(3)=1;

for i=1:msize(1)
    for j=1:msize(2)
        list=ar8(i,j); %intanto ci teniamo a distanza 1 poi casomai si modifica la ar8 per fornire liste giuste.
        [idx,~]=size(list);
        costii=zeros(idx,1); %questo l'ho aggiunto non c'era poi ho bloccato il costii=[];
        for ii=1:idx
           costii(ii,1)=cost(i,j,list(ii,1),list(ii,2)); 
        end
        MM{i,j}=[list,costii];
        %costii=[];
        list=[];
    end
end


                
U=[distance(start(1),start(2),goal(1),goal(2)),0,goal(1),goal(2)];

