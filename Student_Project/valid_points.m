%% valid point check
xc=750*cos(q1(1));zc=500;xw=(PX1-xc);zw=PZ1-zc;rxz=sqrt(xw^2 + zw^2);rxz=abs(rxz);
xw=PX1;yw=PY1;rxy=sqrt(xw^2 + yw^2);rxy=abs(rxy);
out=rxy;
if rxz > 250
    handles.Message.String = num2str('The point is out of work space in XZ Plane, , please enter new coordinates'); 
    out=rxz;
        elseif rxy < 500 
    handles.Message.String = num2str('The point is out of work space in XY Plane, , please enter new coordinates');
    out=rxy;
    elseif rxy > 1000
    handles.Message.String = num2str('The point is out of work space in YZ Plane, , please enter new coordinates');
    out=rxy;
else
   handles.Message.String = num2str(' OK');
   out=rxy;
end
%handles.Message.String = num2str(' Please enter value of Link Length');