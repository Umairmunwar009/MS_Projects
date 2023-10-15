function cols = checkBaseInCircle(Xb,Yb,Xc,Yc,r,clearance,base_radius)
%%  Variable definations
        % Xb Yb; point to be checked
        % Xc,Yc,r position and radies of obstacle
        % base_radius ; radius of base
        % clearance; clearance form base
%% ----------- code -------------
cols = 0;
d = sqrt((Xb - Xc)^2 + (Yb - Yc)^2);
rs = r+base_radius+clearance;

if (d <= rs) || (d == rs)
    cols = 1;
end

end
