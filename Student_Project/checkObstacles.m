function cols = checkObstacles(XA, YA, XB, YB, XC,YC,R,R_C)
%check if one given segment intersect one given obstacle
% returns 1 if collides, 0 if not
% segment AB, disc obstacle : center C, radius R
cols = 0;
R_CC= R_C+ R;

a = ((XB-XA)^2 + (YB-YA)^2);
b = (2*XA*(XB-XA) - 2*XC*(XB-XA) + 2*YA*(YB-YA) - 2*YC*(YB-YA));
c = XA^2 + XC^2 - 2*XA*XC + YA^2 + YC^2 - 2*YA*YC - R_CC^2;
delta = b^2 - 4*a*c;
mid_x=(XA+XB)/2;mid_y=(YA+YB)/2;
fx=(mid_x-XC)^2 + (mid_y-YC)^2 - R_CC^2;

if (delta>=0)  || (fx < 0) || (c < 0)
    t1 = (-b - sqrt(delta))/(2*a);
    t2 = (-b + sqrt(delta))/(2*a);
    if ((t1 >= 0 && t1 <= 1) || (t2 >= 0 && t2 <= 1))
        cols = 1;
    end
end
end

