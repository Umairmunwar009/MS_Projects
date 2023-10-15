% function robot_config( b )
% %OBSTACLE Summary of this function goes here
% %   Detailed explanation goes here

switch b
    case 1  % 2 DOF
        L1=1400;  L2=1200;  L3=000;  L4=000;  L5=000;  L6=0;  d1=0;d2=0;d3=0;d4=0;d5=0;d6=0;
        alpha1=0;alpha2=0;alpha3=0;alpha4=0;alpha5=0;alpha6=0;o1=0;o2=0;o3=0;o4=0;o5=0;o6=0;
        offset_y=1500;
        write_para_dh
        handles.Y_offset.String  = num2str((floor(offset_y)));
    case 2      % 5 DOF
        L1=500;  L2=800;  L3=800;  L4=750;  L5=700;  L6=0;  d1=0;d2=0;d3=0;d4=0;d5=0;d6=0;
        alpha1=0;alpha2=0;alpha3=0;alpha4=0;alpha5=0;alpha6=0;o1=0;o2=0;o3=0;o4=0;o5=0;o6=0;
        offset_y=2500;
        write_para_dh
        handles.Y_offset.String  = num2str((floor(offset_y)));
    case 3          % 5 DOF
        L1=500;  L2=800;  L3=700;  L4=650;  L5=500;  L6=0;  d1=0;d2=0;d3=0;d4=0;d5=0;d6=0;
        alpha1=0;alpha2=0;alpha3=0;alpha4=0;alpha5=0;alpha6=0;o1=0;o2=0;o3=0;o4=0;o5=0;o6=0;
        offset_y=2000;
        write_para_dh
        handles.Y_offset.String  = num2str((floor(offset_y)));
    case 4          % 5 DOF
        L1=600;  L2=800;  L3=900;  L4=900;  L5=950;  L6=0;  d1=0;d2=0;d3=0;d4=0;d5=0;d6=0;
        alpha1=0;alpha2=0;alpha3=0;alpha4=0;alpha5=0;alpha6=0;o1=0;o2=0;o3=0;o4=0;o5=0;o6=0;
        offset_y=3000;
        write_para_dh
        handles.Y_offset.String  = num2str((floor(offset_y)));
        
        
    case 5
          L1=500;  L2=900;  L3=1100;  L4=000;  L5=000;  L6=0;  d1=0;d2=0;d3=0;d4=0;d5=0;d6=0;
        alpha1=0;alpha2=0;alpha3=0;alpha4=0;alpha5=0;alpha6=0;o1=0;o2=0;o3=0;o4=0;o5=0;o6=0;
        offset_y=1200;
        write_para_dh
        handles.Y_offset.String  = num2str((floor(offset_y)));
    case 6
        L1=100;  L2=250;  L3=350;  L4=450;  L5=600;  L6=700;  d1=0;d2=0;d3=0;d4=0;d5=0;d6=0;
        alpha1=0;alpha2=0;alpha3=0;alpha4=0;alpha5=0;alpha6=0;o1=0;o2=0;o3=0;o4=0;o5=0;o6=0;
        write_para_dh
        offset_y=1200;
        handles.Y_offset.String  = num2str((floor(offset_y)));
    case 7
        L1=2500;  L2=1000;  L3=2000;  L4=0;  L5=0;  L6=0;  d1=2000;d2=0;d3=0;d4=0;d5=0;d6=0;
        alpha1=90;alpha2=0;alpha3=0;alpha4=0;alpha5=0;alpha6=0;o1=0;o2=0;o3=0;o4=0;o5=0;o6=0;
        write_para_dh
    case 8
        L1=500;  L2=500;  L3=500;  L4=500;  L5=600;  L6=700;  d1=200;d2=200;d3=-200;d4=0;d5=0;d6=0;
        alpha1=90;alpha2=0;alpha3=0;alpha4=0;alpha5=0;alpha6=0;o1=0;o2=0;o3=0;o4=0;o5=0;o6=0;
        write_para_dh
%     case 9
%         L1=500;  L2=1000;  L3=1000;  L4=1000;  L5=1000;  L6=0;  d1=0;d2=0;d3=0;d4=0;d5=0;d6=0;
%         alpha1=0;alpha2=0;alpha3=0;alpha4=0;alpha5=0;alpha6=0;o1=0;o2=0;o3=0;o4=0;o5=0;o6=0
%         write_para_dh

end

% end

