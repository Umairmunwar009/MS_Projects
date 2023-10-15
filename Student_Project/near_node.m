%% Check for nearest node
function [node,tree,x_new_par,N_p,dist_min,G,dist_g] = near_node(xy,base_radius,near_rad,x_par,goal,dist_0,goal_rad,dist_g_p,DELTA_xx,c)
   
tree=[];node=[];
% x_temp=x_par(1);y_temp=x_par(2);
G=0;N_p=0;
x_new_par=x_par;
dist      =sqrt((xy(1)-x_par(1))^2+(xy(2)-x_par(2))^2);     % distance from parent c'(n_parent)
dist_goal=sqrt((xy(1)-goal(1))^2+(xy(2)-goal(2))^2);        % distance from goal h(n)
% DELTA_xx=0;
d_p=(base_radius + near_rad) - dist;
dist_g = dist_g_p +  dist;              % total g(n)=c(n)+ c'(n_parent cost for reaching node) 
if c==1
   dist_f = dist_goal - DELTA_xx ;       % insert check for f(n) =  h(n) dist_g +  dist_goal; or dist +  dist_goal-DELTA; 
elseif c==2
   dist_f = dist +  dist_goal - DELTA_xx ; % insert check for f(n) = g(n) + h(n) dist_g +  dist_goal; or dist +  dist_goal-DELTA;
end




if (d_p) && (dist_f < dist_0) %&& (xy(1) ~=x_temp || xy(2) ~=y_temp)      % to ensure pt is within near radius %  Criterion; h(n):dist_goal< dist_0 , or f(n)(dist_f< dist_0 )
    N_p=N_p+1;
    tree  = [xy(1), xy(2), dist_g , dist_goal];
    node  = [xy(1), xy(2)];
    x_new_par=[xy(1) xy(2)];                    % Assining current node to parent node
    dist_min = dist_goal;                       % update for criteria :h(n):dist_goal or f(n):dist_f
    dist_0 = dist_min;                          % update with minimum goal distance for next iteration h(n)
    
    if dist_goal < goal_rad
        %                 fprintf('GOAL ACHIEVED with Distance from Goal, h(n)..............:%0.0f \n', dist_min);
        fprintf('GOAL ACHIEVED with herustic distance:...........  f(n)= g(n) + h(n)..:%0.3f \n', (dist_g +  dist_goal-goal_rad));
        G=1;
    end
    
end
dist_min=dist_0;

end