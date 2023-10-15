%% ===============   Corrections Joints angles =========================
if q0(1) < 0 
   q0(1)=pi+q0(1);  % angle changed from p1 to  deg i.e 180
    handles.Message.String = num2str('Joint 1 of start Pos exceed limit');
    handles.out.String = num2str((floor(q0(1))));
end
if q1(1) < 0  
   q1(1)=pi+q1(1);
   handles.Message.String = num2str('Joint 1 is < 0');
    handles.out.String = num2str((floor(q1(1))));
end
if q1(2) < 0  && L2 >0
   q1(2)=pi+q1(2);
   handles.Message.String = num2str('Joint 2 is < 0');
    handles.out.String = num2str((floor(q1(2))));
end
if q1(3) < 0   && L3 >0
   q1(3)=pi+q1(3);
   handles.Message.String = num2str('Joint 3 is < 0 ');
    handles.out.String = num2str((floor(q1(3))));
end
if q1(4) < 0 && L4 >0  
   q1(4)=pi+q1(4);
   handles.Message.String = num2str('Joint 4 is < 0');
    handles.out.String = num2str((floor(q1(4))));
end

if q1(5) < 0   && L5 >0
   q1(5)=pi+q1(5);
   handles.Message.String = num2str('Joint 5 is < 0');
    handles.out.String = num2str((floor(q1(5))));
end

if q1(6) < 0   && L6 >0
   q1(6)=pi+q1(6);
   handles.Message.String = num2str('Joint 6 is < 0');
    handles.out.String = num2str((floor(q1(6))));
end
