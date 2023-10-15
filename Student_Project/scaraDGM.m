function Xef = scaraDGM( L1,L2,theta1,theta2 )
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here


phi1= theta1;
phi2= theta1+theta2;
phi1= theta1+theta2++theta3;
phi1= theta1+theta2+theta3+theta4;
phi1= theta1+theta2+theta3+theta4+theta5;
phi1= theta1+theta2+theta3+theta4+theta5+theta6;
Xef = [ L1*cos(phi1) + L2*cos(phi2);
        L1*sin(phi1) + L2*sin(phi2)];

end

