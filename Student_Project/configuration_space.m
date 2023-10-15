%% This program calculate congiguration space
% figure(99)
figure('Name','Configuration Space');
L1=1;L2=1;
th1_lim=180;th2_lim=150;
% axis([0 5 0 5]);
% title(['Link 1 =',num2str(L1),'m' ';   Link 2 = ' num2str(L2),'m']);
for theta1 =0:1:th1_lim
    for theta2 =0:1:th2_lim
        figure(99)
    XY = scaraDGM( L1,L2,theta1,theta2 );
        plot(XY(1),XY(2),'*');hold on;
    end
end
clear L1 L2 th2_lim th2_lim XY