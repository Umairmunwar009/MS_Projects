function [ cost ] = path_cost( x,y )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
cost =0;
for i =1:length(x)-1
   
p1 = [x(i) y(i)];
p2 = [x(i+1) y(i+1)];
cost  = cost + eculidein( p1,p2 );

end

