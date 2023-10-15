
%% Ploting base location & circles

plot(base_opt(1),base_opt(2),'or');xlabel('X-Cordinate'); ylabel('Y_cordinate');%hold on
      % ploting circles
   R=total_length;
   circle(base_opt, R);%hold on