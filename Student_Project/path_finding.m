%% Main Program for path finding in 2D

% clf(handles.axes6);
%%  load parameters
init_goal      % Loading Initial & Goal Points & optimiation parameters
near_rad=near_rad_xx;
band=band_xx;    
DELTA_xx=DELTA;
r_rew=200;       % near radius & random # gen max limit              
wt_init=wt;
a = get(handles.obstacle_selection,'Value');
    [Obstacle] = obstacle(a);
c = get(handles.Path_Search_Method,'Value');
if (c==3)
 %DEFINE THE 2-D MAP ARRAY
MAX_X=500;
MAX_Y=500;
MAX_VAL=1000;
%This array stores the coordinates of the map and the 
%Objects in each coordinate
MAP=2*(ones(MAX_X,MAX_Y));
axes(handles.axes6);
% Obtain Obstacle, Target and Robot Position
% Initialize the MAP with input values
% Obstacle=-1,Target = 0,Robot=1,Space=2
j=0;
x_val = 1;
y_val = 1;
axis([1 MAX_X+1 1 MAX_Y+1])
grid on;
grid minor

hold on;
n=0;%Number of Obstacles

% BEGIN Interactive Obstacle, Target, Start Location selection
pause(0);
%h=msgbox('Please Select the Target using the Left Mouse button');
%uiwait(h,5);
% if ishandle(h) == 1
%     delete(h);
% end
% xlabel('Please Select the Target using the Left Mouse button','Color','black');
% but=0;
% while (but ~= 1) %Repeat until the Left button is not clicked
%     [xval,yval,but]=ginput(1);
% end
xval=str2double(handles.x_goal.String);
yval=str2double(handles.y_goal.String);
xval=floor(xval);
yval=floor(yval);
xTarget=xval;%X Coordinate of the Target
yTarget=yval;%Y Coordinate of the Target

MAP(xval,yval)=0;%Initialize MAP with location of the target
plot(xval+.5,yval+.5,'gd');
text(xval+1,yval+.5,'Target')

%pause(2);
% h=msgbox('Select Obstacles using the Left Mouse button,to select the last obstacle use the Right button');
%   xlabel('Select Obstacles using the Left Mouse button,to select the last obstacle use the Right button','Color','blue');
% uiwait(h,10);
% if ishandle(h) == 1
%     delete(h);
% end
% ang=0:0.01:2*pi; 
% xp=50*cos(ang)+500;
% yp=50*sin(ang)+500;
%      xval=floor(xp);
%      yval=floor(yp);
%      MAP(xp(2),yp(2))=-1;%Put on the closed list as well
y1=400:-1:150;
x1=ones(1,251)*200;

y2=50:1:500;
x2=ones(1,451)*150;


x3=400:-1:301;
y3=ones(1,100)*400;

y4=400:-1:301;
x4=ones(1,100)*300;
% axis([-10 100 -10 100])
plot(x2,y2) 
hold on
%plot(x2,y2) 
% plot(x2,y2) 
% plot(x3,y3) 
% plot(x4,y4)
% while but == 1
%     [xval,yval,but] = ginput(1);
%     xval=floor(xval);
%     yval=floor(yval);
% for i=1:1:length(x1)
%     MAP(x1(i),y1(i))=-1;%Put on the closed list as well
%     plot(x1,y1,'r');
% %     plot(xp,yp,'ro');
%  end%End of While loop

 for i=1:1:length(x2)
    MAP(x2(i),y2(i))=-1;%Put on the closed list as well
    plot(x2,y2,'r');
%     plot(xp,yp,'ro');
 end%End of While loop
% for i=1:1:length(x1)
%     MAP(x1(i),y1(i))=-1;%Put on the closed list as well
%     plot(x1,y1,'r');
% %     plot(xp,yp,'ro');
%  end%End of While loop
  
 
%  for i=1:1:length(x1)
%     MAP(x3(i),y3(i))=-1;%Put on the closed list as well
%     plot(x3,y3,'r');
% %     plot(xp,yp,'ro');
%  end%End of While loop
%  
%  for i=1:1:length(x1)
%     MAP(x4(i),y4(i))=-1;%Put on the closed list as well
%     plot(x4,y4,'r');
% %     plot(xp,yp,'ro');
%  end%End of While loop
%pause(1);

%h=msgbox('Please Select the Vehicle initial position using the Left Mouse button');
%uiwait(h,5);
% if ishandle(h) == 1
%     delete(h);
% end
% xlabel('Please Select the Vehicle initial position ','Color','black');
% but=0;
% while (but ~= 1) %Repeat until the Left button is not clicked
%     [xval,yval,but]=ginput(1);
%     xval=floor(xval);
%     yval=floor(yval);
% end
xval=str2double(handles.x_initial.String);
yval=str2double(handles.y_initial.String);
xStart=xval;%Starting Position
yStart=yval;%Starting Position
MAP(xval,yval)=1;
 plot(xval+.5,yval+.5,'bo');
%End of obstacle-Target pickup

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%LISTS USED FOR ALGORITHM
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%OPEN LIST STRUCTURE
%--------------------------------------------------------------------------
%IS ON LIST 1/0 |X val |Y val |Parent X val |Parent Y val |h(n) |g(n)|f(n)|
%--------------------------------------------------------------------------
OPEN=[];
%CLOSED LIST STRUCTURE
%--------------
%X val | Y val |
%--------------
% CLOSED=zeros(MAX_VAL,2);
CLOSED=[];
tic
%Put all obstacles on the Closed list
k=1;%Dummy counter
for i=1:MAX_X
    for j=1:MAX_Y
        if(MAP(i,j) == -1)
            CLOSED(k,1)=i; 
            CLOSED(k,2)=j; 
            k=k+1;
        end
    end
end
CLOSED_COUNT=size(CLOSED,1);
%set the starting node as the first node
xNode=xval;
yNode=yval;
OPEN_COUNT=1;
path_cost=0;
goal_distance=distance(xNode,yNode,xTarget,yTarget);
OPEN(OPEN_COUNT,:)=insert_open(xNode,yNode,xNode,yNode,path_cost,goal_distance,goal_distance);
OPEN(OPEN_COUNT,1)=0;
CLOSED_COUNT=CLOSED_COUNT+1;
CLOSED(CLOSED_COUNT,1)=xNode;
CLOSED(CLOSED_COUNT,2)=yNode;
NoPath=1;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% START ALGORITHM
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
while((xNode ~= xTarget || yNode ~= yTarget) && NoPath == 1)
%  plot(xNode+.5,yNode+.5,'go');
 exp_array=expand_array(xNode,yNode,path_cost,xTarget,yTarget,CLOSED,MAX_X,MAX_Y);
 exp_count=size(exp_array,1);
 %UPDATE LIST OPEN WITH THE SUCCESSOR NODES
 %OPEN LIST FORMAT
 %--------------------------------------------------------------------------
 %IS ON LIST 1/0 |X val |Y val |Parent X val |Parent Y val |h(n) |g(n)|f(n)|
 %--------------------------------------------------------------------------
 %EXPANDED ARRAY FORMAT
 %--------------------------------
 %|X val |Y val ||h(n) |g(n)|f(n)|
 %--------------------------------
 for i=1:exp_count
    flag=0;
    for j=1:OPEN_COUNT
        if(exp_array(i,1) == OPEN(j,2) && exp_array(i,2) == OPEN(j,3) )
            OPEN(j,8)=min(OPEN(j,8),exp_array(i,5)); %#ok<*SAGROW>
            if OPEN(j,8)== exp_array(i,5)
                %UPDATE PARENTS,gn,hn
                OPEN(j,4)=xNode;
                OPEN(j,5)=yNode;
                OPEN(j,6)=exp_array(i,3);
                OPEN(j,7)=exp_array(i,4);
            end%End of minimum fn check
            flag=1;
        end%End of node check
%         if flag == 1
%             break;
    end%End of j for
    if flag == 0
        OPEN_COUNT = OPEN_COUNT+1;
        OPEN(OPEN_COUNT,:)=insert_open(exp_array(i,1),exp_array(i,2),xNode,yNode,exp_array(i,3),exp_array(i,4),exp_array(i,5));
    end%End of insert new element into the OPEN list
 end%End of i for
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 %END OF WHILE LOOP
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 %Find out the node with the smallest fn 
  index_min_node = min_fn(OPEN,OPEN_COUNT,xTarget,yTarget);
  if (index_min_node ~= -1)    
   %Set xNode and yNode to the node with minimum fn
   xNode=OPEN(index_min_node,2);
   yNode=OPEN(index_min_node,3);
   path_cost=OPEN(index_min_node,6);%Update the cost of reaching the parent node
  %Move the Node to list CLOSED
  CLOSED_COUNT=CLOSED_COUNT+1;
  CLOSED(CLOSED_COUNT,1)=xNode;
  CLOSED(CLOSED_COUNT,2)=yNode;
  OPEN(index_min_node,1)=0;
  else
      %No path exists to the Target!!
      NoPath=0;%Exits the loop!
  end;%End of index_min_node check
end;%End of While Loop
%Once algorithm has run The optimal path is generated by starting of at the
%last node(if it is the target node) and then identifying its parent node
%until it reaches the start node.This is the optimal path

i=size(CLOSED,1);
Optimal_path=[];
xval=CLOSED(i,1);
yval=CLOSED(i,2);
i=1;
Optimal_path(i,1)=xval;
Optimal_path(i,2)=yval;
i=i+1;

if ( (xval == xTarget) && (yval == yTarget))
    inode=0;
   %Traverse OPEN and determine the parent nodes
   parent_x=OPEN(node_index(OPEN,xval,yval),4);%node_index returns the index of the node
   parent_y=OPEN(node_index(OPEN,xval,yval),5);
   
   while( parent_x ~= xStart || parent_y ~= yStart)
           Optimal_path(i,1) = parent_x;
           Optimal_path(i,2) = parent_y;
           %Get the grandparents:-)
           inode=node_index(OPEN,parent_x,parent_y);
           parent_x=OPEN(inode,4);%node_index returns the index of the node
           parent_y=OPEN(inode,5);
           i=i+1;
   end
 j=size(Optimal_path,1);
 %Plot the Optimal Path!
 p=plot(Optimal_path(j,1)+.5,Optimal_path(j,2)+.5,'bo');
 j=j-1;
 for i=j:-1:1
  pause(.05);
  set(p,'XData',Optimal_path(i,1)+.5,'YData',Optimal_path(i,2)+.5);
 drawnow ;
 end
 plot(Optimal_path(:,1)+.5,Optimal_path(:,2)+.5);
else
 pause(.05);
 h=msgbox('Sorry, No path exists to the Target!','warn');
 uiwait(h,5);
end

h1 = msgbox({'Path Finding Time for A* in seconds.........' '' num2str(toc)} ,'Info');

%CASE 3 End here   

else
   
%% Initialization of parameters
min_cost_t=1000000;
near_rad_max=0;GG=0;ck_rew=0;
path_node= zeros(50, 2);path_tree = zeros(50, 4);
%% ploting workspace & obstacles
axes(handles.axes6);
plot_workspace( work_space,Obstacle,init,r_init,goal,goal_rad )
tog_val_goal = get(handles.dynamic_button,'value');
tog_val_obs = get(handles.Dynamic_Obstacle_button,'value');
%% Iterations for Path finding
% vx = -0.8;vy = -0.0;vz = 0;
vx =str2double(handles.vx.String)/5;vy =str2double(handles.vy.String)/5;vz =str2double(handles.vz.String)/5;
vx_o =str2double(handles.vx_o.String)/10;vy_o =str2double(handles.vy_o.String)/10;vz_o =str2double(handles.vz_o.String)/10;
% set(groot,'defaultAxesLineStyleOrder',{'-*',':','o'})
for tt=1:NN
    handles.iter_NN.String = num2str((floor(tt)));
    G=0;x_b=init(1);y_b=init(2);
     fprintf('# of path finding iteration in process...............................................:%0.0f \n',tt);
    %% plot random points
    speed=0.0000000000001;
    %    speed=0.000005;
    x_par=init;x_new_par=init;dist_0=5000;
    tree=[];node=[];tree_t=[];node_t=[];tree_length=0;tree_t_rew =[];
    valid_pt=0; data=[];
    dist_g_p=0; check_i = 40000;                        % Inintializing as on start pt g(n)=0;
    temperature = 0;
    tic
    for i=1:1:No_Rand
        
        if tog_val_goal == 1
            [ new_pos,tog_val_goal ] = dynamic(vx,vy,vz,goal,num_range,goal_rad );
            goal = new_pos;
%             circle([goal(1) goal(2)],goal(4));
            viscircles ([goal(1) goal(2)],goal(4),'Color','g');

        end
        if tog_val_obs == 1
            [new_Obstacle, tog_val_obs] = dynamic_obs(vx_o,vy_o,vz_o,Obstacle,num_range);
            Obstacle = new_Obstacle;
            viscircles(Obstacle(:,1:2),Obstacle(:,3));
        end
        
        near_rad=wt*near_rad_xx;band=wt*band_xx;
        handles.cur_n_band.String = num2str((floor(band)));
        handles.cur_n_rad.String = num2str((floor(near_rad)));
        
        x1(1) = random('Normal',x_b,band,1,1); % -2000,2000
        x1(2) = random('Normal',y_b,band,1,1); % -2000,2000
        
        if x1(1)> num_range(1) && x1(2)> num_range(2) && x1(1)< num_range(3) && x1(2)< num_range(4) %&& G == 0
            sm=0;
            [inObstacle] = col_check(x_b,y_b,x1,Obstacle,clearance,base_radius,sm);
            if  inObstacle == 0
                [node,tree,x_new_par,N_p,dist_min,G,dist_g] = near_node(x1,base_radius,near_rad,x_par,goal,dist_0,goal_rad,dist_g_p,DELTA_xx,c);
                %                    plot(x1(1),x1(2),'*g');hold on;
                if N_p>0
                    
                    plot(x1(1),x1(2),'.b');hold on;
%                     circle(x1,band); %% new line added to show random circle
                    x_par=x_new_par;    %  comment if rewire is used
                    
                    valid_pt=valid_pt+1;
                    dist_0 = dist_min;  % Criteria function mimimum goal or f(n) distance from previous iteration for use in near node func
                    dist_g_p=dist_g;    % Tocal Cost (g) update from previous iteration for use in near node func
                    
%                   n_temperature = i-temperature;    % temperature , no of random points generated to find path
                    tree_t  = [tree_t; [tree(1), tree(2), dist_g, dist_min, temperature ,wt,near_rad,band,DELTA_xx ]];   %% [x ,y ,cost, dist from goal, min_dist, temperature]
                    node_t  = [node_t; [node(1), node(2)]];
                    x_b=x1(1);y_b=x1(2);
                                       
                    wt_temp=wt;

                    if wt > wt_init
                        if temperature> 0 && temperature<=25
                            temperature= 20;
                            wt= wt - fact*temperature;
                            DELTA_xx= DELTA_xx-fact_d*temperature;
                        elseif temperature> 25 && temperature<=50
                            temperature=50;
                            wt=wt - fact*temperature;
                            DELTA_xx= DELTA_xx-fact_d*temperature;
                        elseif temperature> 50 && temperature<=75
                            temperature=75;
                            wt=wt - fact*temperature;
                            DELTA_xx= DELTA_xx-fact_d*temperature;
                        elseif temperature> 75 && temperature<=100
                            temperature= 100;
                            wt=wt - fact*temperature;
                            DELTA_xx= DELTA_xx-fact_d*temperature;

                        elseif temperature> 100
                            temperature= 150;
                            wt=wt - fact*temperature;
                            DELTA_xx= DELTA_xx-fact_d*temperature;
                        end
                        
                        if wt < wt_init
                            %                 else
                            wt=wt_init;
                            DELTA_xx= DELTA;
                            temperature=temperature/50;
                        end
                    end
                    
                else
                    temperature=temperature+1;
                    wt=wt+fact*temperature;
                    DELTA_xx= DELTA_xx+fact_d*temperature;
                    
                end
                
                handles.Curr_weight.String = num2str((floor(wt)));
                handles.curr_delta.String = num2str((floor(DELTA_xx)));
                handles.temperature.String = num2str((floor(temperature)));
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
        %         wt=wt+fact;
        %         DELTA_xx= DELTA_xx*fact_d;
        
        if i == check_i
            fprintf('.....Current Random Number in process.................................:%0.0f \n',check_i);
            
            check_i = check_i+10000;
            handles.Message.String = num2str('..... Path finding inprocess...............', i);
        end
        
        handles.Message.String = num2str(' Path finding inprocess.....', i);
        handles.random.String = num2str((floor(i)));
        
        
        
    end
    
    if G==0
        fprintf('GOAL NOT ACHIEVED even after iterations..........:%0.f \n',iter);
        fprintf('Distance from Goal...............................:%0.3f \n',dist_min);
        fprintf('Number of node Points............................:%0.3f \n',valid_pt);
        fprintf('Allowed Random Number within radius..............:%0.3f \n',near_rad);
        fprintf('Max Near Radius due to Weight....................:%0.4f \n',near_rad_max);
        handles.Message.String = num2str('...!!!!  GOAL NOT FOUND  !!!.....');
    else
        fprintf('Number of node Points...............................:%0.0f \n',valid_pt);
        fprintf('Total Cost upto Goal location........................................:%0.4f \n',dist_g);
        fprintf('Starting Random Number within radius................:%0.3f \n',near_rad);
        fprintf('Allowed Distance from Goal..........................:%0.3f \n',goal_rad);
        fprintf('Actual Distance from Goal location..................:%0.4f \n',dist_min);
        fprintf('Number of random number generated to achieve Goal...:%0.0f \n',iter);
        handles.Message.String = num2str('...!!!! GOAL FOUND  !!!.....');
        GG=GG+1;
        if dist_g < min_cost_t
            min_cost_t=dist_g;
            path_node = node_t;
            path_tree = tree_t;
            handles.min_cost.String = num2str((floor(min_cost_t)));
        end
        
    end
    
    near_rad_max=0;
    
%     figure (12);
%     title(['Near Radius =',num2str(near_rad),'m','Max Near Radius = ',num2str(near_rad_max),'m','Rand # = ',num2str(i) ,'DELTA_xx = ',num2str(DELTA_xx) ]);
%     plot_workspace( work_space,Obstacle,init,r_init,goal,goal_rad )
%   edge=line([x_par(1) x_new_par(1)],[x_par(2) x_new_par(2)], 'LineWidth', 1);
%   plot(x1(1),x1(2),'om');hold on;
%     plot(node_t(:,1),node_t(:,2),'-g','LineWidth',1);
    
    
end
fprintf('Shortest Path Cost................................................:%0.4f \n',min_cost_t);
fprintf('Total # of Iterations.............................................:%0.0f \n',NN);
fprintf('Goal Achieved in iterations.......................................:%0.0f \n',GG);

% plot(path_node(:,1),path_node(:,2),'--ms',...
%     'LineWidth',1,...
%     'MarkerSize',1,...
%     'MarkerEdgeColor','c',...
%     'MarkerFaceColor',[0.5,0.5,0.5]);hold on

if GG>0
    handles.Message.String = num2str('...!!!! GOAL FOUND  !!!.....');
end

toc
h1 = msgbox({'Path Finding Time in seconds.........' '' num2str(toc)} ,'Info');
handles.path_time.String = num2str((floor(toc)));
uiwait(h1,10)

global X;
global Y;

axes(handles.axes6)
title('Selected Path');
edge=line([x_par(1) x_new_par(1)],[x_par(2) x_new_par(2)], 'LineWidth', 2);
% plot(path_node(:,1),path_node(:,2),'kd');hold on;
% plot(path_node(:,1),path_node(:,2),'-b','LineWidth',2);

%% new lines added for data saving
% save('path.mat','x','y')
%% path smoothing
x=path_node(:,1);y=path_node(:,2);
X=x;Y=y;
bf_pts = length(x);
handles.bf_pts.String = num2str((floor(bf_pts)));
%% smoothing algo
% axes(handles.axes6)
% iter_smooth       = str2double(handles.iter_smooth.String);
% smooth_per_edit   = str2double(handles.smooth_per_edit.String);
% [ n_pts, pts, xx , yy ] = path_smooth( x , y,iter_smooth,Obstacle,clearance,base_radius,smooth_per_edit );
% [ pathcost ] = path_cost( xx,yy );
% handles.del_pts.String = num2str((floor(pts)));
% handles.n_pts.String = num2str((floor(n_pts)));
% toc
% X =xx;Y=yy;
% handles.pathcost.String = num2str((floor(pathcost)));
% plot( xx, yy,'-g','LineWidth',2,'MarkerFaceColor','k', 'MarkerSize',12 ); hold on;
% plot(xx,yy,'m+');

% h1 = msgbox({'Total Time including smoothing in seconds.........' '' num2str(toc)} ,'Info');
% handles.path_time.String = num2str((floor(toc)));
% uiwait(h1,10)
%% end of smoothing allgo
%% ploting wts
% tree_t  = [tree_t; [tree(1), tree(2), dist_g, dist_min, temperature ,wt,near_rad,band,DELTA_xx ]];  

figure('Name','Results') 
subplot(2,2,1)
plot(path_tree(:,5),'-b','LineWidth',1);hold on;
title('Subplot 1:Tempertaure');

subplot(2,2,2)
plot(path_tree(:,6),'-r','LineWidth',1);hold on;
title('Subplot 2:Weight');

subplot(2,2,3)
plot(path_tree(:,7),'-g','LineWidth',1);hold on;
title('Subplot 3:Near Radius');

subplot(2,2,4)
plot(path_tree(:,8),'-y','LineWidth',1);hold on;
title('Subplot 4:Band');
end
