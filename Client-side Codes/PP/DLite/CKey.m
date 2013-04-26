function [KEY]=CKey(i,j)
global start M km
%start è la posizione attuale del robot...

KEY=[min(M{i,j}(1),M{i,j}(2))+distance(start(1),start(2),i,j)+km,min(M{i,j}(1),M{i,j}(2))];
%actually  START STANDS FOR ACTUAL POSITION OF THE ROBOT%
