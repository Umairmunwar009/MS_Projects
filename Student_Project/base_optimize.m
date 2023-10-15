%% Base optimization  
workshop_workspace
H=sqrt(RX^2 + Ry^2);
L=Wg+H;L1=L/2;L2=L/2;

%%  down position
    xbase_opt=rx+(rl)/2;
     b=sqrt((total_length)^2-(xbase_opt-rx)^2);
     ybase_opt=(ry+rw)- abs(b);
     base_opt=[xbase_opt ybase_opt];
     plot_base    
     %%  up position
    xbase_opt=rx+(rl)/2;
     b=sqrt((total_length)^2-(xbase_opt-rx)^2);
     ybase_opt=(ry)+ abs(b);
     base_opt=[xbase_opt ybase_opt];
     plot_base 
     %%  Right position
     ybase_opt=ry+(rw)/2;
     b=sqrt((total_length)^2-(ybase_opt-ry)^2);
     xbase_opt=(rx)+ abs(b);
     base_opt=[xbase_opt ybase_opt];
     plot_base
     %%  left position
     ybase_opt=ry+(rw)/2;
     b=sqrt((total_length)^2-(ybase_opt-ry)^2);
     xbase_opt=(rx+rl)- abs(b);
     base_opt=[xbase_opt ybase_opt];
     plot_base 