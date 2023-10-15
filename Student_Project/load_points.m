
% Load Point to file
% inter=str2double(handles.inter.String);

PX=str2double(handles.Pos_X.String);PY=str2double(handles.Pos_Y.String);PZ=str2double(handles.Pos_Z.String);% Home
PX1=str2double(handles.PX1.String);PY1=str2double(handles.PY1.String);PZ1=str2double(handles.PZ1.String);   % Point 1
PX2=str2double(handles.PX2.String);PY2=str2double(handles.PY2.String);PZ2=str2double(handles.PZ2.String);   % Point 2
PX3=str2double(handles.PX3.String);PY3=str2double(handles.PY3.String);PZ3=str2double(handles.PZ3.String);   % Point 3
PX4=str2double(handles.PX4.String);PY4=str2double(handles.PY4.String);PZ4=str2double(handles.PZ4.String);   % Point 4
PX5=str2double(handles.PX5.String);PY5=str2double(handles.PY5.String);PZ5=str2double(handles.PZ5.String);   % Point 5

if PY1 > PY_L || PY2 > PY_L || PY3 > PY_L || PY4 > PY_L ||PY5 > PY_L
    handles.Message.String = num2str('Y Cordinate point is out of configuration space , , please enter new coordinate');
else
    handles.Message.String = num2str('All Cordinates, Ok');
end

if PZ1 > PZ_L || PZ2 > PZ_L || PZ3 > PZ_L || PZ4 > PZ_L ||PZ5 > PZ_L
    handles.Message.String = num2str('Z Cordinate point is out of configuration space , , please enter new coordinate');
    else
    handles.Message.String = num2str('All Cordinates, Ok');
end

if PX1 > PX_L || PX2 > PX_L || PX3 > PX_L || PX4 > PX_L ||PX5 > PX_L
    handles.Message.String = num2str('X Cordinate point is out of configuration space , , please enter new coordinate');
    else
    handles.Message.String = num2str('All Cordinates, Ok');
end