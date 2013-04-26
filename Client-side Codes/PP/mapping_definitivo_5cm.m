%FILE DI MAPPATURA                                                        %
clear all
clear workspace
%INIZIALIZZAZIONE%
%conoscenza nulla dell'ambiente, tutto nero quindi tutto zero 
load('Mapp.mat');
l_t_1=zeros(120,120);
l_t_p=zeros(120,120);
l_t=zeros(120,120);
BEL=zeros(120,120);
colormap(gray);                             %mappa in scala di grigi      %
pxa=0;
pya=0;                            
%carica dati mappatura%
load('J.mat');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for n=1:40                              %n-esimo passo                     %
    mes=J{n};
    th=mes(:,1)*(pi/180);                   %step angolari                %
    r=mes(:,2)*100;                         %misure per i-esimo step      %
    l_p=0.5*ones(120,120);
    for i=1:180                             %i-esimo step angolare        %
        thi=th(i);
        c=0;
        while (abs(pxa))<60 && pya<(119-n) %verifico che pixels          %
            c=c+1;                          %corrispondono ad ogni raggio.%
            rc(c)=5*c;                      %genero r per punti           %
            px=rc(c)*cos(thi);
            py=2*(n-1)+rc(c)*sin(thi);
            pxmap(c)=fix(px/5);             %riconduco tutto alla misura  %
            pymap(c)=fix(py/5);             %dei pixel, in due vettori.   %
            pxa=pxmap(c);
            pya=pymap(c);
        end
        dis=abs(rc-r(i));                   %verifico quale pixel si      %
        [dx,index]=min(dis);                %avvicina di più alla misura. %
        for s=1:(index-1)                   %assegno a tutti i pixel      %
            ii=120-pymap(s);                %antecedenti il valore di     %
            jj=pxmap(s)+60;                %spazio vuoto.                %
            l_p(ii,jj)=0.8;
        end
        s=index;                            %assegno al pixel più vicino  %
        ii=120-pymap(s);                    %il valore di spazio pieno.   %
        jj=pxmap(s)+60;
        l_p(ii,jj)=0.1;
        pxa=0;
        pya=0;
        pxmap=0;   
        pymap=0;
        rc=0;
        end
    for i=1:120                             %per i pixel calcolo l_t e bel%
        for j=1:120
           l_t(i,j)=log(l_p(i,j)/(1-l_p(i,j)))+l_t_1(i,j);
           BEL(i,j)=1-1/(1+exp(l_t(i,j)));  %300x300 da 0 a 1             %
        end
    end
        
        l_t_1=l_t;                          %l_t-1 del passo successivo è %
        BEL=BEL*100;                        %ugale all'l_t di adesso.     %           
        image(BEL)                          %genero immagine              %
        grid on
        grid minor
        pause (0.1)
end
