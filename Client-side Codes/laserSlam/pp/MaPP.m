function Mapp=MaPP(BEL,cols,rows)

%%Mapper for the PP%%
%%A Program By Edoardo Cacciavillani%%
%%ILTP%%
%%A97501678%%

Mapp=ones(rows,cols);
for i=1:rows
    for j=1:cols
        if BEL(i,j)<0.1
            Mapp(i,j)=0;
        else
            Mapp(i,j)=1;
        end
    end
end


%save('Mapp.mat','Mapp')
%image(Mapp*100)                         
%grid on
%grid minor