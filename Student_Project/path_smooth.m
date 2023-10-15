function [ n_pts, pts, xx , yy ] = path_smooth( x , y, iter_smooth,Obstacle,clearance,base_radius,smooth_per_edit )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

n_pts =length(x);tot=length(x);
iter=0;
pts = 0;
xx = []; yy = [];
h = waitbar(0,'Please wait... smoothing in process');
while iter < iter_smooth

    for i = 1: n_pts - 3
        if i< (n_pts - 3)
            p1 = [x(i)  , y(i)];
            p2 = [x(i+1), y(i+1)];
            p3 = [x(i+2), y(i+2)];
            
            [ dist_1_3 ] = abs(eculidein( p1,p3 ));
            [ dist_1_2 ] = abs(eculidein( p1,p2 ));
            [ dist_2_3 ] = abs(eculidein( p2,p3 ));
            
            %% to check any point withing twp pts within obstacles
            x_b= p1(1);y_b=p1(2);x1 =p3;
            if dist_1_3 < 10
                sm=0;
            elseif dist_1_3 < 30 && dist_1_3 >= 10
                sm =1;
            elseif dist_1_3 < 500 && dist_1_3 >= 30
                sm =2;
            elseif dist_1_3 < 800 && dist_1_3 >= 500
                sm =3;
            elseif dist_1_3 < 1500 && dist_1_3 >= 800
                sm =4;

            elseif dist_1_3 < 2000 && dist_1_3 >= 1500
                sm =5;
            else
                sm =10;
            end
           [ inObstacle ] = col_check(x_b,y_b,x1,Obstacle,clearance,base_radius,sm);
%%
            if dist_1_3 < (dist_1_2 + dist_2_3) && inObstacle == 0  %&& dist_1_3 < 400
                x(i+2) = x(i+3);     y(i+2) = y(i+3);
                x(i+3) = [];         y(i+3) = [];
                pts = pts + 1;
                n_pts = n_pts -1;
                progress = (tot-length(xx))/tot;
                if   progress > 0.24 && progress < 0.26
                    progress =0.25;
                    waitbar(progress)
                elseif progress > 0.48 && progress < 0.52
                    progress =0.5;
                    waitbar(progress)
                elseif progress > 0.93 && progress < 0.95
                    progress =0.75;
                    waitbar(progress)
                elseif progress > 0.991 && progress < 1.0
                    progress =1.0;
                    waitbar(progress)
                end
            end
            
            
        end
        
        xx =x; yy =y;
        iter = iter +1;
        if n_pts/tot*100 < smooth_per_edit
            iter = iter_smooth;
        end
    end
     
end
close(h)
end

