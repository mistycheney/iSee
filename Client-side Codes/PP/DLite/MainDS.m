function MainDS (Mapp,pos,goal)
%slast = sgoal ricorda questo nell'uso delle variabili
global M U start %start è dinamico e coincide con la posizione iniziale, poi del robot


Initialize(Mapp,pos,goal);
PATH=ComputePath(pos, goal, km);
PATH(1,:)=[pos(1),pos(2)];
counter=1;
cn=[1:8,1];
while PATH(counter)~=goal
    %qui ci andrebbe un go to con istruzioni al robot%
%    list=M{PATH(counter,1),PATH(counter,2)}(5);
%    for i=1:8
%    trip(i)=M{list(i,1),list(i,2)}(1)+cost(PATH(counter,1),PATH(counter,2),list(i,1),list(i,2));
%    end
%    PATH(counter+1)=min(trip);
%    counter=counter+1;
%    trip=[];
% scan for graph change cost 
% should ew use a map or a function that give us immediatly a list of
% changed values. I hypotize to use a list file with all the xx,yy
% coordinate of the change in cost
if changed ~= []
km=km+distance(pos(1),pos(2),start(1),start(2));

    for i=1:size(changed)
    cold=M{changed(i,1),changed(1,2)}(5) 

    for ii=1:8
    cn(ii)=cost(changed(i,1),changed(i,2),cold(ii,1),cold(ii,2));
    end
    cnew=[cold(:,1:2),cn];
    M{changed(i,1),changed(1,2)}(5)=cnew;
    for ii=1:8
        
        if cold(ii,3)>cnew(ii,3)
            if [changed(i,1),changed(i,2)]~=goal
                M{changed(i,1),changed(i,2)}(2)=min(M{changed(i,1),changed(i,2)}(2),cnew(ii,3)+M{cnew(ii,1),cnew(ii,2)}(1));
            end

        elseif M{changed(i,1),changed(i,2)}(2)==cold(ii,3)+M{cnew(ii,1),cnew(ii,2)}(1)
            if [changed(i,1),changed(i,2)]~=goal 
                M{changed(i,1),changed(i,2)}(2)=minsurrounding(changed(i,1),changed(i,2)); %%molto dispendioso e non ottimizzato... pensa ad un'ottimizzazione
            end
        end
               
    end
     %FAI MOLTA ALTTENZIONE QUESTO UpVertex dentro o fuori dal for potrebbe cabiare MOLTO!!!!!!!!!!!!!!!!!!!!!!!!
    end
    UpVertex(changed(:,1),changed(:,2));
end
ComputePath
end
    