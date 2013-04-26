function [PATH] = ComputePath(goal)
%verifica cosa pu� essere usato come global.

global M MM U km start %M � una variabile globale

while (U(1,1:2)-CKey(start(1),start(2))<=0) | (M{start(1),start(2)}(2)>M{start(1),start(2)}(1))%%??? va scritta differentemente
    KeiLimit=CKey(start(1),start(2))
    %rhs=M{start(1),start(2)}(2)
    %g=M{start(1),start(2)}(1)
    kold=U(1,1:2) %kold=U.Top
    x=U(1,3)
    y=U(1,4)
    knew=CKey(x,y) 
    if (kold(1)<knew(1)) || (kold(1)==knew(1) && kold(2)<knew(2)) 
        ifCP=1
        U(1,:)=[knew(1),knew(2),x,y]; %qui andrebbe l'inserimento con heap
        U=sortrows(U); %questo fa perdere tempo.
    elseif (M{x,y}(1)>M{x,y}(2))
        elseifCP=1
        M{x,y}(1)=M{x,y}(2);
        U(1,:)=[];%elimina U.Pop
        M{x,y}(3)=0;
        U=U;
        list=MM{x,y}; %list = surrounding cells
        [ix,iy]=size(list);
        for i=1:ix
            if list(i,1)~=goal(1) || list(i,2)~=goal(2)
                checker=1
                M{list(i,1),list(i,2)}(2)=min(M{list(i,1),list(i,2)}(2),cost(list(i,1),list(i,2),x,y)+M{x,y}(1));
                UpVertex(list(i,1),list(i,2)) %update al vertex sorrounding
            end
        end
    else
        elseCP=1
    gold=M{x,y}(1);
    M{x,y}(1)=inf;
    list=MM{x,y};
    list=[list;[x,y,0]]; %% vertcat... rivedi!!!!!!!!!!!!!!!!!!
    [ix,iy]=size(list);
        for i=1:ix
            if M{list(i,1),list(i,2)}(2)==cost(list(i,1),list(i,2),x,y)+gold; %% qui HAI MODIFICATO UGUALE CON DIVERSO!!!!!
                if list(i,1)~=goal(1) || list(i,2)~=goal(2)
                    
                    rhslisybef=M{list(i,1),list(i,2)}(2)
                    ifinelse=1
                    
                    follower=MM{list(i,1),list(i,2)};
                    [iix,iiy]=size(follower);
                    for ii=1:iix
                         %gruppo dei succ di s
                        cpart(ii)=cost(list(i,1),list(i,2),follower(ii,1),follower(ii,2))+M{follower(ii,1),follower(ii,2)}(1);
                    end
                    M{list(i,1),list(i,2)}(2)=min(cpart);
                    cpart=[];
                end
            end
        listx=list(i,1)
        listy=list(i,2)
        MM3=M{list(i,1),list(i,2)}(3)
        rhslisyaft=M{list(i,1),list(i,2)}(2)
        UpVertex(list(i,1),list(i,2));
        end
    end
end

%calculate path

PATH=[];
counter=1;
PATH(counter,:)=[start(1),start(2)];
a33=M{3,3}(:);
a34=M{3,4}(:);
a44=M{4,4}(:);
a46=M{4,6}(:);
a36=M{3,6}(:);
a65=M{6,5}(:)
a54=M{5,4}(:)
a66=M{6,6}(:)
while (PATH(counter,1)~=goal(1) || PATH(counter,2)~=goal(2)) & (counter<10) 
    list=MM{PATH(counter,1),PATH(counter,2)};
    [idx,idy]=size(list);
    for i=1:idx
        ddddddddd=M{list(i,1),list(i,2)}(1);
        costi(i,:)=[ddddddddd+cost(PATH(counter,1),PATH(counter,2),list(i,1),list(i,2)),list(i,1),list(i,2)];
    end
    counter=counter+1;
    costf=sortrows(costi);
    %rgrgr=PATH(counter-1,:);
    if costf(1,2)~=PATH(counter-1,1) || costf(1,3)~=PATH(counter-1,2) 
    PATH(counter,:)=costf(1,2:3);
    gcgcgc=1;
    else
    PATH(counter,:)=costf(2,2:3);
    gcgcgcgc=2;
    end
end

        
   
            

            
    
