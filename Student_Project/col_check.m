function [ inObstacle ] = col_check(x_b,y_b,x1,Obstacle,clearance,base_radius,sm)
%OBSCTACLE Summary of this function goes here

num_obst = size(Obstacle,1);
inObstacle = 0;
if sm==0
    n = 5;
elseif sm==1
    n=50;
elseif sm==2
    n=2000;
elseif sm==3
    n=8000;
elseif sm==4
    n=12000;
elseif sm==5
    n=15000;
else
    n= 18000;
end

xx = linspace(x_b,x1(1),n);yy = linspace(y_b,x1(2),n);
for obst=1:num_obst
    for ii = 1:n
        %inObstacle_check = checkBaseInCircle(x1(1),x1(2),Obstacle(obst,1), Obstacle(obst,2),Obstacle(obst,3),clearance,base_radius);
        inObstacle_check = checkBaseInCircle(xx(ii),yy(ii),Obstacle(obst,1), Obstacle(obst,2),Obstacle(obst,3),clearance,base_radius);
        inObstacle = inObstacle + inObstacle_check;
    end
end
end


