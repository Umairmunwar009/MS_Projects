function [ x_new_par_up , cost_min ] = rewire(xx,xy,x_new_par,x_par,tree_t,r,tree_length)
%REWIRE Summary of this function goes here
%   Detailed explanation goes here
% detal for tree component |||| tree_t  = [tree_t; [tree(1), tree(2), dist_g, dist_min ]];   %% [x ,y ,cost, dist from goal]
% tree_length = 0;
x=0;y=0;t=0;
cost_min=100000000;
% tree_length = length(tree_t(:,1))
d_par = sqrt((xx-x_par(1))^2+(xy-x_par(2))^2);     % distance from preious nodes c(n_parent)
cost_par = tree_t(tree_length,3) + d_par;           % cost through current parent c + through x_par

for i= 1:1:tree_length
    
    x = tree_t(i,1);y = tree_t(i,2);
    d = sqrt((xx-x)^2+(xy-y)^2);                   % distance from neighbour temp parent c'(n_parent)
    
    if d < r
        cost_new = tree_t(i,3) + d;                              % cost through neighbouring node c + through x_neighbor
        if (cost_new < cost_par) && (cost_new < cost_min)
            x_p=[xx xy];
            %                 circle (x_p, r);
            cost_min = cost_new;
            t=i;
            x_new_par_up = [x,y];
            plot(x,y,'*r');
     
        else
            x_new_par_up=x_new_par;
        end
    else
        x_new_par_up=x_new_par;
    end
    
end
       
end



