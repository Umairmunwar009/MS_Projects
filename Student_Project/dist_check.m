function [PB_check,PB_d]= dist_check(PB,Obstacle, base_radius)
num_PB = size(PB,1);
num_obst = size(Obstacle,1);
PB_check=[];PB_d=[];x_temp=0;y_temp=0;N_p=0;xb=0;yb=0;
 for i=1:num_PB
     
 for obst=1:num_obst
 
    xb=PB(i,1);yb=PB(i,2);   
       x= Obstacle(obst,1);
       y= Obstacle(obst,2);
       r= Obstacle(obst,3);
       dist=sqrt((x-xb)^2+(y-yb)^2);
       rad_dist=base_radius+r;
       d=dist-rad_dist;
       if d>0
           
           if xb ~=x_temp || yb ~=y_temp
%                 figure (99);plot(xb,yb,'ob');hold on;% self
                x_temp=xb;
                y_temp=yb;
                N_p=N_p+1;
                PB_check  = [PB_check; [xb, yb, dist,rad_dist,d,N_p,obst]];
                PB_d=[PB_d; [xb, yb]];
          
            end
       end
       
  end
 end
end