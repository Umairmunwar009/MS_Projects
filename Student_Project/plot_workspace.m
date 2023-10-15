function plot_workspace( work_space,Obstacle,init,r_init,goal,goal_rad )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
a = [work_space(1)-50 work_space(1)+ work_space(3) + 50 work_space(2)-50 work_space(2)+ work_space(4) + 50];
axis(a);hold on;
rectangle('Position',work_space,'EdgeColor','m','LineWidth',2);

viscircles(Obstacle(:,1:2),Obstacle(:,3));
circle(init,r_init);
circle(goal,goal_rad);
end

