% each row corresponds to one row of DH table i.e. theta, d, 
% a and alpha parameters of each joint 
% for DH table [joint_Angle link_offset link_length Link_twist]
L1=1000;L2=1000;
dh = [ 0 0 L2 0; 
       0 0 L2 0];
        
r = SerialLink(dh)
r.tool
r.gravity
r.base
%r.teach
th1=0;th2=0;%th3=0;th4=0;
q0=[th1 th2];% th3 th4];
T0=r.fkine(q0);
th1=0;th2=0;%th3=-pi/4;th4=-pi/4;
qf=[th1 th2];% th3 th4];
Tf=r.fkine(qf);
%gggggggggggggggg
temp=0:2:50;
q=jtraj(q0,qf,temp);

% Traj
start=[T0.t(1), T0.t(2),T0.t(3)];
last=[Tf.t(1), Tf.t(2),Tf.t(3)];
via=[750 0 250; 0 750 250; -500 0 250; last];
qf1=r.ikine(T, [0 0 0], 'mask',[1 1 1 0 0 0])*180/pi;
x = mstraj(via, [1 1 1], [], start, 0.1, 1); % via points, velocity, start point, timestep, acceleration
%x=1000*x;
figure(5);plot3 (x(:,1), x(:,2),x(:,3));hold on;
r.plot(q);
%ttttttttttttttttttttttttttttttt


%iiiiiiiiiii

[Q,QD,QDD] = jtraj(q0,qf, temp, [0 0 0 0], [1 10 10 10]);% initial and final joint velocity for the trajectory and a time vector.
 
%Ts=ctraj(TA, TB, 50);

 %[x xd]=jtraj(q0,qf2, 50, [0 0], [10 10]); %specify initial and final velocity 
    figure(10); plot(Q);title('Joint Angle vs. time');xlabel('Time(s)');ylabel('Joint Angle (deg/s)')
     figure(11);plot(QD);title('Joint Velocites vs. time');xlabel('Time(s)');ylabel('Joint Velocity (mm/s)')
     figure(12);plot(QDD);title('Joint Acceleration vs. time');xlabel('Time(s)');ylabel('Joint Acceleration (mm/s^2)')
      %figure(13);qplot(q)
        %figure(14);qplot(Q)
%pause(0.25);

%RTBPose.animate (Tf);
%returns the pose of end-effector
%syms Px;syms Py;syms Pz;

