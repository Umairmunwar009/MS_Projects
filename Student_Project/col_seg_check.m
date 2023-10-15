function [ colsr ] = col_seg_check(x1, x_new_par,Obstacle,clearance,base_radius)
%OBSCTACLE Summary of this function goes here
%   Detailed explanation goes here

R_C= clearance + base_radius;

    num_obst = size(Obstacle,1);
         
     colsr=0;
     for obst=1:num_obst
         % checking if segment intersect an object
         cols1 = checkObstacles(x1(1),x1(2),x_new_par(1),x_new_par(2), Obstacle(obst,1),...
             Obstacle(obst,2),Obstacle(obst,3),R_C);
             colsr = colsr + cols1;
     end
    
         
 end