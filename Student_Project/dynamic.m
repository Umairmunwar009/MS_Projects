function [ new_pos,tog_val ] = dynamic(vx, vy,vz,goal,num_range,goal_rad )

 xd = vx;yd = vy;zd = vz;
 new_pos = [goal(1)+xd, goal(2)+yd, goal(3)+zd,goal_rad ];
 tog_val=1;
 if new_pos(1)< num_range(1), new_pos(1) = num_range(1); tog_val=0;end
 if new_pos(2)< num_range(2), new_pos(2) = num_range(2); tog_val=0;end
 if new_pos(1)> num_range(3), new_pos(1) = num_range(3); tog_val=0;end
 if new_pos(2)> num_range(4), new_pos(2) = num_range(4); tog_val=0;end
   
            
end