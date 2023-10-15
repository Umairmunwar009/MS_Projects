% Workspace for Robot manipulation
% This programs also finds robot base location w.r.t workspace area

%%  
offset_y=str2double(handles.Y_offset.String); % read y offset from panel
yy = offset_y;                              %base offset
xbase_opt=0; ybase_opt=0; zbase_opt=0;
rl = 2000;                              %rectangle length as per workspace area
rw = 2000;                              %rectangle width as per workspace area
%%
read_para_dh
total_length=PX_L;

r(1).x=xbase_opt- rl/2;r(2).x=xbase_opt + rl/2; % pt 1 x lower left, pt 2 lower right
r(4).x=xbase_opt- rl/2;r(3).x=xbase_opt + rl/2; % pt 4 x upper left, pt 3 upper right

% yy= abs(sqrt(total_length^2-(xbase_opt-r(2).x)^2)) + rw - adj; %  offset y
TL= abs(sqrt((xbase_opt-r(2).x)^2 + (rw + yy)^2)); %  offset y
if TL> total_length
    fprintf('Link Lengths are NOT OK, Required lengths should be >:%0.4f \n',total_length);
    fprintf('Distance from maximum reach .........................:%0.4f \n',TL);
end

r(1).y=ybase_opt + yy     ;r(2).y=ybase_opt + yy;    % pt 1 y lower left, pt 2 lower right
r(4).y=ybase_opt + yy + rw;r(3).y=ybase_opt + yy + rw; % pt 4 y upper left, pt 3 upper right

r(1).z=0; r(2).z=r(1).z ;r(3).z = r(1).z;r(4).z = r(1).z;

%% Worspace coordinates
     P1=[r(1).x  r(1).y  r(1).z];          % Worspace bottom Left  Corner 
     P2=[r(2).x  r(2).y  r(2).z];          % Worspace bottom Right Corner 
     P3=[r(3).x  r(3).y  r(3).z];          % Worspace Upper Right  Corner 
     P4=[r(4).x  r(4).y  r(4).z];          % Worspace Upper Left Corner 
     P =[0 (ybase_opt+yy) 0];               % Worspace Home 
%      PP= [r(1).x  r(1).y  r(1).z];        % Worspace Center 


%% end of changes -----------------------------------------------------------------------------------------------
% Rx = 1000;                  %Workspace x position inside the workshop (bottom left corner is the origin)
% Ry = 1000;                  %Workspace y position inside the workshop (bottom left corner is the origin)
% Rx_origin = -1000;
% Ry_origin = -1000;          %Workspace origin
% rx=Rx+Rx_origin;
% ry=Ry+Ry_origin;
% rz=0;

% rg = 100;                 %rectangle discretize grid size
%% Worspace coordinates
%      PP1=[rx ry rz];        % Worspace bottom Left  Corner 
%      PP2=[rx+rl ry rz];     % Worspace bottom Right Corner 
%      PP3=[rx+rl ry+rw rz];  % Worspace Upper Right  Corner 
%      PP4=[rx ry+rw rz];     % Worspace Upper Left Corner 
%      PP=[rx+rl/2 ry+rw/2 rz];% Worspace Center 
    %% Optimization of robot base
%      read_para_dh

%      %% down position
%      xbase_opt=rx+(rl)/2;
%      b=sqrt((total_length)^2-(xbase_opt-rx)^2);
%      ybase_opt=(ry+rw)- abs(b);
%      zbase_opt=0;
%      base_opt=[xbase_opt ybase_opt zbase_opt];
%         
%      %% Rectangle coordinates w.r.t to robot base reference
%      P =[PP(1)- base_opt(1) PP(2)- base_opt(2) zbase_opt];% Home position end effector
%      P1=[PP1(1)-base_opt(1) PP1(2)-base_opt(2) zbase_opt]; 
%      P2=[PP2(1)-base_opt(1) PP2(2)-base_opt(2) zbase_opt]; 
%      P3=[PP3(1)-base_opt(1) PP3(2)-base_opt(2) zbase_opt];
%      P4=[PP4(1)-base_opt(1) PP4(2)-base_opt(2) zbase_opt];
 
  