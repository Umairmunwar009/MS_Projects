%% ===============   check points =========================
function check_points(PX,PY,PZ,PX_L,PY_L,PZ_L)


if PY > PY_L 
    handles.Message.String = num2str('Y Cordinate point is out of configuration space , , please enter new coordinate');

elseif PX > PX_L
    handles.Message.String = num2str('X Cordinate point is out of configuration space , , please enter new coordinate');

elseif PZ > PZ_L
    handles.Message.String = num2str('Z Cordinate point is out of configuration space , , please enter new coordinate');
    
else
    handles.Message.String = num2str('Coordinates are OK');
end
end

% if PY1 > PY_limit || PY2 > PY_limit || PY3 > PY_limit || PY4 > PY_limit ||PY5 > PY_limit
%     handles.Message.String = num2str('Y Cordinate point is out of configuration space , , please enter new coordinate');