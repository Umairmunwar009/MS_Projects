% each row corresponds to one row of DH table i.e. theta, d, 
% a and alpha parameters of each joint 
% for DH table [joint_Angle link_offset link_length Link_twist]
%L1=500;L2=500;L3=500;L4=500;
% inter=str2double(handles.inter.String);

inter=10;
 L1=100;  L2=200;  L3=300;  L4=200;  L5=150;  L6=100;  d1=0;d2=0;d3=0;d4=0;d5=0;d6=0;
 alpha1=0;alpha2=90;alpha3=0;alpha4=0;alpha5=0;alpha6=0;o1=0;o2=0;o3=0;o4=0;o5=0;o6=0;

% read_para_dh
    dh = [ o1/180*3.14 d1 L1 alpha1/180*3.14; 
           o2/180*3.14 d2 L2 alpha2/180*3.14;
           o3/180*3.14 d3 L3 alpha3/180*3.14; 
           o4/180*3.14 d4 L4 alpha4/180*3.14;
           o5/180*3.14 d5 L5 alpha5/180*3.14;
           o6/180*3.14 d6 L6 alpha6/180*3.14];
   rob.name = 'K_Robot';
   rob = SerialLink(dh);
   