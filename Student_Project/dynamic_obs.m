function [new_Obstacle, tog_val_obs] = dynamic_obs(vx,vy,vz,Obstacle,num_range)

new_Obstacle = [Obstacle(1)+vx, Obstacle(2)+vy, Obstacle(3)]; tog_val_obs=1;
if (new_Obstacle(1))< num_range(1), new_Obstacle(1) = num_range(1)+new_Obstacle(3); tog_val_obs=0;end
if (new_Obstacle(2))< num_range(2), new_Obstacle(2) = num_range(2)+new_Obstacle(3); tog_val_obs=0;end
if (new_Obstacle(1))> num_range(3), new_Obstacle(1) = num_range(3)-new_Obstacle(3); tog_val_obs=0;end
if (new_Obstacle(2))> num_range(4), new_Obstacle(2) = num_range(4)-new_Obstacle(3); tog_val_obs=0;end
end