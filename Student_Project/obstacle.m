function [ Obstacle] = obstacle( a )
%OBSTACLE Summary of this function goes here
%   Detailed explanation goes here

switch a
    case 1
        Obstacle    = [-600    -400         200; -150    -150       200; 300    300         200;650    600      200];
        %       Obstacle_c  = [-600    -400+ofy     200; -150    -150+ofy   200; 300    300+ofy     200;650    600+ofy  200];
    case 2
        Obstacle = [-600     400    100;    0    -350   100; 300    600   200;650    -600   200];
    case 3
        Obstacle = [-600    -400    200; -150    -150   200; 300    300   200;650    600   200];
    case 4
        Obstacle = [-600    -400    200;    0    -350    200; 300    300  200;650    600   200;650    -200   200];
    case 5
        Obstacle = [-600    -400    200;    0    -350    200; 300    300  200;650    600   200;650    -200   200;-200    450   200];
    case 6
        Obstacle = [-600    -400    200;    0    -250    200; 300    350  200;650    700   200;650    -200   200;-200    450   200; -400    -750   170];
    case 7
        Obstacle = [-600    -400    200;    0    -250    200; 300    350  200;650    700   200;650    -200   200;-200    450   200; -400    -700   170];
    case 8
        Obstacle = [-600    -400    200;    0    -250    200; 300    350  200;650    700   200;-750    10    200;-200    450   200; -400    -720   170];
    case 9
        Obstacle = [-600    -400    200; -100     -0    200; 300    350  200;650    700   200;-700    60    200;-200    450   200; -400    -700   170];
    case 10  % wall type obstacles
        r=70;
        Obstacle = [0     -900    r;0    -800  r;0    -700  r;0   -600   r;0   -500   r; 0  -400   r;0   -300   r;0   -200   r;0    -100   r;
                    0       0     r;0    100   r;0    200   r;0    300   r;0    400   r;0    500   r;0    600   r;0    700   r];
    case 11  % wall type obstacles
        r=90;
        Obstacle = [-600     -700    r;-550   -600  r;-500  -500  r;-450   -400   r;-400   -300   r;-350  -200   r;-300  -100   r;-250   0   r;-200    100   r;
                    -150      200    r;-100    300  r;-50    300   r;0    400   r;50    500   r;100    600   r;150    700   r];%0    700   r];
        
    case 12  % for moving obstacles
       r1=200;
         Obstacle = [700     400    r1];
         
    case 13  % for moving obstacles
         y2=50:1:500;
         x2=ones(1,451)*150;

         
         T=ones(3,451);
         T(1,:)=x2;
         T(2,:)=y2;
         T(3,:)=1;
         Obstacle = T';
         
    case 14
        radius_1 = 70;
        xCenter_1 = 50;
        yCenter_1 = 30;
        theta = linspace(0, 360, 4*pi*radius_1); % More than needed to avoid gaps.
        x_1 = xCenter_1 + radius_1 * cosd(theta);
        y_1 = yCenter_1 + radius_1 * sind(theta);

        x_round_1 = floor(x_1);
        y_round_1 = floor(y_1);    
        
         T=ones(3,879);
         T(1,:)=x_round_1;
         T(2,:)=y_round_1;
         T(3,:)=radius_1;
         Obstacle = T
 
end

