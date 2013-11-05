function dist = distance(x1,y1,x2,y2)
%This function calculates the distance between any two cartesian 
%coordinates.
dist=sqrt(((x1-x2)*10)^2 + ((y1-y2)*10)^2);

%si potrebbe usare round(sqrt((x1-x2)^2 + (y1-y2)^2));
