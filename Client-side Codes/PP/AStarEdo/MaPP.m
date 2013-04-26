function Mapp=MaPP(BEL)

%%Mapper for the PP%%
%%A Program By Edoardo Cacciavillani%%
%%ILTP%%
%%A97501678%%
load('BEL.mat');
MAP=ones(120,120);
for i=1:120
    for j=1:120
        if BEL(i,j)>0.95
            Mapp(i,j)=1;
        else
            Mapp(i,j)=0;
        end
    end
end
save('Mapp.mat','Mapp')
image(Mapp*100)                         
grid on
grid minor