
%% This m file loads initial & Goal Points alonwith its radius
base_radius=5;clearance=35;rg=10;      % robot base & clearance from obstacle
work_space = [-1000 -1000 2000 2000]; % workspace rectangle
num_range = [(work_space(1)+clearance) (work_space(2)+clearance) (work_space(1)+work_space(3) - clearance) (work_space(2)+work_space(4) - clearance)];

NN=str2double(handles.NN.String);
No_Rand=100000;                         % max # of random #generated in each iteration


%% initial & goal parameters

offset_y=str2double(handles.Y_offset.String);

init_x = str2double(handles.x_initial.String);
init_y = str2double(handles.y_initial.String);
init_z = str2double(handles.z_initial.String);
r_init = str2double(handles.r_init.String);

goal_x = str2double(handles.x_goal.String);
goal_y = str2double(handles.y_goal.String);
goal_z = str2double(handles.z_goal.String);
goal_rad = str2double(handles.goal_rad.String);

init = [init_x init_y init_z r_init];
goal = [goal_x goal_y goal_z goal_rad];

%% Optimization parameters

  near_rad_xx   = str2double(handles.near_radius.String)/10;   % cirular radius of near radius & randomness
  band_xx       = str2double(handles.random_band.String)/10;
  fact          = str2double(handles.rand_fact.String)/100;     % wt used for near radius & random # gen if no near pt found
  wt            = str2double(handles.rand_wt.String)/10;     % wt used for near radius & random # gen if no near pt found
  DELTA         = str2double(handles.delta_heru.String)/100;  % delta added  in heuristic func to comes out of local minima & factor if stucked
  fact_d        = str2double(handles.delta_fact.String)/100;
  near_rad_limit= str2double(handles.n_rad_limit.String);




