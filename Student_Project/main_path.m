%% Main Program for path finding in 2D
%% Paramerts to be adjusted
    NN=5;
near_rad_xx=4;band_xx=4;fact=3.0;wt=1.0;No_Rand=100000 ;DELTA=0.5;fact_d=1.05;DELTA_xx=DELTA;
    near_rad_limit=260;r_rew=200;
    
  % near_rad_xx=5;band_xx=5;fact=4.0;wt=1.0;No_Rand=50000 ;DELTA=1;fact_d=1.09;DELTA_xx=DELTA;near_rad_limit=290;
  % near_rad_xx=15;band_xx=15;fact=5.0;wt=1.0;No_Rand=50000 ;DELTA=4;fact_d=1.05;DELTA_xx=DELTA;near_rad_limit=300;
  % near_rad_xx=20;band_xx=20;fact=0.20;wt=1.0;No_Rand=100000;DELTA=5;fact_d=1.10;DELTA_xx=DELTA;
  % optimized parameters
near_rad=near_rad_xx;band=band_xx;
init= [-900 -900];goal=[900 900];
r_init=50;goal_rad=80;
base_radius=10;clearance=35;rg=50;

%%
% a = get(handles.obstacle_selection,'Value');
%     [Obstacle] = obstacle(a);
%%
% Obstacle = [-600    -400    200; -150    -150   200; 300    300   200;650    600   200];
% Obstacle = [-600     400    100;    0    -350   100; 300    600   200;650    -600   200];
% Obstacle = [-600    -400    200; -150    -150   200; 300    300   200;650    600   200];
% Obstacle = [-600    -400    200;    0    -350    200; 300    300  200;650    600   200;650    -200   200];
% Obstacle = [-600    -400    200;    0    -350    200; 300    300  200;650    600   200;650    -200   200;-200    450   200];
% Obstacle = [-600    -400    200;    0    -250    200; 300    350  200;650    700   200;650    -200   200;-200    450   200; -400    -750   170];
% Obstacle = [-600    -400    200;    0    -250    200; 300    350  200;650    700   200;650    -200   200;-200    450   200; -400    -700   170];
% Obstacle = [-600    -400    200;    0    -250    200; 300    350  200;650    700   200;-750    10    200;-200    450   200; -400    -720   170];
Obstacle = [-600    -400    200; -100     -0    200; 300    350  200;650    700   200;-700    60    200;-200    450   200; -400    -700   170];
%% definations for class path
min_cost_t=1000000;
near_rad_max=0;GG=0;ck_rew=0;
path_node= zeros(50, 2);path_tree = zeros(50, 4);
for tt=1:NN
    G=0;x_b=init(1);y_b=init(2);
        fprintf('# of path finding iteration in process...............................................:%0.0f \n',tt);
    %% plot random points
    speed=0.000005;
    x_par=init;x_new_par=init;dist_0=5000;
    tree=[];node=[];tree_t=[];node_t=[];tree_length=0;tree_t_rew =[];
    valid_pt=0; data=[];
    dist_g_p=0; check_i = 40000;                        % Inintializing as on start pt g(n)=0;
        
    for i=1:1:No_Rand
        near_rad=wt*near_rad_xx;band=wt*band_xx;
        if near_rad> near_rad_limit
            wt=1.0;
        end
        if near_rad> near_rad_max
            near_rad_max=near_rad;
        end
        x1(1) = random('Normal',x_b,band,1,1); % -2000,2000
        x1(2) = random('Normal',y_b,band,1,1); % -2000,2000
        
        if x1(1)> -1000 && x1(2)> -1000 && x1(1)< 1000 && x1(2)< 1000 %&& G == 0
            [inObstacle] = col_check(x1,Obstacle,clearance,base_radius);
            if  inObstacle == 0
                [node,tree,x_new_par,N_p,dist_min,G,dist_g] = near_node(x1,base_radius,near_rad,x_par,goal,dist_0,goal_rad,dist_g_p,DELTA_xx);
%                    plot(x1(1),x1(2),'*g');hold on;
               if N_p>0 %&& G == 0
                    [colsr] = col_seg_check(x1, x_new_par,Obstacle,clearance,base_radius);
                    if colsr == 0
%                         figure (11);title(['Near Radius =',num2str(near_rad),'m','Max Near Radius = ',num2str(near_rad_max),'m','Rand # = ',num2str(i) ,'DELTA_xx = ',num2str(DELTA_xx) ]);
%                         axis([-1050 1050 -1050 1050]);hold on;
%                         rectangle('Position',[-1000 -1000 2000 2000],'EdgeColor','b','LineWidth',2);
%                         viscircles(Obstacle(:,1:2),Obstacle(:,3));
%                         circle(init,r_init);circle(goal,goal_rad);
%                         edge=line([x_par(1) x_new_par(1)],[x_par(2) x_new_par(2)], 'LineWidth', 2);
%                         plot(x1(1),x1(2),'om');hold on;
                          %% Rewire
%                           if ck_rew > 1 && valid_pt > 2
%                               tree_length = length(tree_t(:,1));
%                               [x_new_par_up , cost_min ] = rewire(x1(1),x1(2),x_new_par,x_par,tree_t,r_rew,tree_length); % new line for rewire
%                               
%                               figure (13);title(['Near Radius =',num2str(near_rad),'m','Max Near Radius = ',num2str(near_rad_max),'m','Rand # = ',num2str(i) ,'DELTA_xx = ',num2str(DELTA_xx) ]);
%                               axis([-1050 1050 -1050 1050]);hold on;
%                               rectangle('Position',[-1000 -1000 2000 2000],'EdgeColor','b','LineWidth',2);
%                               viscircles(Obstacle(:,1:2),Obstacle(:,3));
%                               circle(init,r_init);circle(goal,goal_rad);
%                               edge=line([x_new_par_up(1) x_par(1)],[x_new_par_up(2) x_par(2)], 'LineWidth', 2);
%                               plot(x1(1),x1(2),'om');hold on;
%                               tree_t_rew  = [ tree_t_rew , ; [x_new_par_up(1), x_new_par_up(2), cost_min ]];   %% [x ,y ,cost, dist from goal]
%                               x_par = x_new_par_up;                                            % new line for rewire
%                               dist_0 = cost_min;
%                           else
%                               x_par=x_new_par;
%                           end
%                           ck_rew = ck_rew + 1;
                            x_par=x_new_par;    %  comment if rewire is used
%% 
                       
                        
                        valid_pt=valid_pt+1;
                        dist_0 = dist_min;  % Criteria func tion mimimum goal or f(n) distance from previous iteration for use in near node func
                        dist_g_p=dist_g;    % Tocal Cost (g) update from previous iteration for use in near node func
                        
                        %LgdBase=['iter = ', num2str(i),'; Near Rad = ', num2str(near_rad),'; # of pos = ', num2str(valid_pt),' ; Goal Radius = ', num2str(goal_rad),' ; Goal Dist = ', num2str(dist_min),' X = ',num2str(x_par(1)),' ; Y = ',num2str(x_par(1))];
                        %legend(num2str(LgdBase))
                        tree_t  = [tree_t; [tree(1), tree(2), dist_g, dist_min ]];   %% [x ,y ,cost, dist from goal]
                        node_t  = [node_t; [node(1), node(2)]];
                        
                        x_b=x1(1);y_b=x1(2);
%                         path_node = node_t;
%                         path_tree = tree_t;
%                         data = [data; NN, node(1) , node(2), i , wt , near_rad , band];
                        wt_temp=wt;
                        wt=1;DELTA_xx=DELTA;
                        near_rad=near_rad_xx;band=band_xx;
                    end
                    
                    
                end
                
                pause(speed)
            end
            
        end
        if G==1
            iter=i;
            fprintf('Weight of Randomness..........................................:%0.4f \n',wt_temp);
            fprintf('Max Near Radius due to Weight.................................:%0.4f \n',near_rad_max);
            fprintf('Band Randomness due to Weight.................................:%0.4f \n',band);
            break
            
        end
        iter=i;
        wt=wt+fact;
        DELTA_xx= DELTA_xx*fact_d;
        
        if i == check_i
          fprintf('.....Current Random Number in process.................................:%0.0f \n',check_i); 
          
            check_i = check_i+10000;
        end
        
        handles.Message.String = num2str(' Path finding inprocess.....', i);
    end
    
    if G==0
        fprintf('GOAL NOT ACHIEVED even after iterations..........:%0.f \n',iter);
        fprintf('Distance from Goal...............................:%0.3f \n',dist_min);
        fprintf('Number of node Points............................:%0.3f \n',valid_pt);
        fprintf('Allowed Random Number within radius..............:%0.3f \n',near_rad);
        fprintf('Max Near Radius due to Weight....................:%0.4f \n',near_rad_max);
    else
        fprintf('Number of node Points...............................:%0.0f \n',valid_pt);
        fprintf('Total Cost upto Goal location.......................:%0.4f \n',dist_g);
        fprintf('Starting Random Number within radius................:%0.3f \n',near_rad);
        fprintf('Allowed Distance from Goal..........................:%0.3f \n',goal_rad);
        fprintf('Actual Distance from Goal location..................:%0.4f \n',dist_min);
        fprintf('Number of random number generated to achieve Goal...:%0.0f \n',iter);
        GG=GG+1;
        if dist_g < min_cost_t 
            min_cost_t=dist_g;
            path_node = node_t;
            path_tree = tree_t;
            
        end
              
    end

    near_rad_max=0;  
      
% figure (12);
title(['Near Radius =',num2str(near_rad),'m','Max Near Radius = ',num2str(near_rad_max),'m','Rand # = ',num2str(i) ,'DELTA_xx = ',num2str(DELTA_xx) ]);
axis([-1050 1050 -1050 1050]);hold on;
rectangle('Position',[-1000 -1000 2000 2000],'EdgeColor','m','LineWidth',2);
viscircles(Obstacle(:,1:2),Obstacle(:,3));
circle(init,r_init);circle(goal,goal_rad);
edge=line([x_par(1) x_new_par(1)],[x_par(2) x_new_par(2)], 'LineWidth', 2);

plot(x1(1),x1(2),'om');hold on;
plot(node_t(:,1),node_t(:,2),'-b','LineWidth',2);  


% figure (20);
% title('Rewire Tree');
% axis([-1050 1050 -1050 1050]);hold on;
% rectangle('Position',[-1000 -1000 2000 2000],'EdgeColor','m','LineWidth',2);
% viscircles(Obstacle(:,1:2),Obstacle(:,3));
% circle(init,r_init);circle(goal,goal_rad);
% % edge=line([x_par(1) x_new_par(1)],[x_par(2) x_new_par(2)], 'LineWidth', 2);
% 
% % plot(x1(1),x1(2),'om');hold on;
% plot(tree_t_rew(:,1),tree_t_rew(:,2),'-r','LineWidth',3);



      
end
fprintf('Shortest Path Cost................................................:%0.4f \n',min_cost_t);
fprintf('Total # of Iterations.............................................:%0.0f \n',NN);
fprintf('Goal Achieved in iterations.......................................:%0.0f \n',GG);

    plot(path_node(:,1),path_node(:,2),'--ms',...
    'LineWidth',3,...
    'MarkerSize',2,...
    'MarkerEdgeColor','c',...
    'MarkerFaceColor',[0.5,0.5,0.5]);hold on
    

% smooth_data( path_node )   % function to be written to smooth data

figure (33);
title('Selected Path');
axis([-1050 1050 -1050 1050]);hold on;
rectangle('Position',[-1000 -1000 2000 2000],'EdgeColor','k','LineWidth',2);
viscircles(Obstacle(:,1:2),Obstacle(:,3));
circle(init,r_init);circle(goal,goal_rad);
edge=line([x_par(1) x_new_par(1)],[x_par(2) x_new_par(2)], 'LineWidth', 2);
plot(x1(1),x1(2),'om');hold on;
plot(path_node(:,1),path_node(:,2),'-b','LineWidth',2);hold on;
%% smoothing filter added
figure (33);
title('Selected Path');
axis([-1050 1050 -1050 1050]);hold on;
rectangle('Position',[-1000 -1000 2000 2000],'EdgeColor','k','LineWidth',2);
viscircles(Obstacle(:,1:2),Obstacle(:,3));
circle(init,r_init);circle(goal,goal_rad);
xxx=path_node(:,1);yyy=path_node(:,2);
[fitresult, gof] = createFit(xxx, yyy);
hold off;
 