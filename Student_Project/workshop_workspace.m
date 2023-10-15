% Work shop workspace

Wl = 4000;          %WorkShop length
Ww = 4000;          %WorkShop width
Wg = 200;           %Workshop discretize grid size

Rx = 1500;          %rectangle x position inside the workshop (bottom left corner is the origin)
Ry = 1500;          %rectangle y position inside the workshop (bottom left corner is the origin)
Rx_origin = 500;Ry_origin = 500; % rectangle origin
rx=Rx+Rx_origin;ry=Ry+Ry_origin;
rl = 900;           %rectangle length
rw = 700;           %rectangle width
rg = 100;           %rectangle discretize grid size
%% Rectangle coordinates
     PP1=[rx ry];PP2=[rx+rl ry];PP3=[rx+rl ry+rw];PP4=[rx ry+rw];PP=[rx+rl/2 ry+rw/2];
    %% Optimization of robot base
% dh_para
    read_para_dh
     % optimum base location
     %L1=500;L2=450;L3=0;L4=0;
     total_length=L1+L2+L3+L4+L5+L6;
     %% down position
     xbase_opt=rx+(rl)/2;
     b=sqrt((total_length)^2-(xbase_opt-rx)^2);
     ybase_opt=(ry+rw)- abs(b);
     base_opt=[xbase_opt ybase_opt];
        
     %% Rectangle coordinates w.r.t to robot base reference
     P =[PP(1)- base_opt(1) PP(2)- base_opt(2)];% position end effector
     P1=[PP1(1)-base_opt(1) PP1(2)-base_opt(2)]; 
     P2=[PP2(1)-base_opt(1) PP2(2)-base_opt(2)]; 
     P3=[PP3(1)-base_opt(1) PP3(2)-base_opt(2)];
     P4=[PP4(1)-base_opt(1) PP4(2)-base_opt(2)];
 
  