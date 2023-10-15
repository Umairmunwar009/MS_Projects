 %% Work Space Plots
figure (11);
     title('Work Space');axis([0 5000 0 5000]);
     rectangle('Position',[Rx_origin Ry_origin Wl Ww]);% Workshop Area
     rectangle('Position',[rx ry rl rw])% Workspace Area
     hold on;
     
     