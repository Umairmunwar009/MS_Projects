function  plot_obstacle( init,goal,r_init, goal_rad,Obstacle )
%PLOT_OBSTACLE Summary of this function goes here
%   plots obstacels with inital & goal points

% clf;

figure (1);title(' Obstaces, Intial & Goal Points');
axis([0 1050 0 1050]);hold on;
rectangle('Position',[-1000 -1000 2000 2000],'EdgeColor','b','LineWidth',2);
viscircles(Obstacle(:,1:2),Obstacle(:,3));
circle(init,r_init);circle(goal,goal_rad);
% edge=line([x_new_par_up(1) x_par(1)],[x_new_par_up(2) x_par(2)], 'LineWidth', 2);
% plot(x1(1),x1(2),'om');hold on;
end

