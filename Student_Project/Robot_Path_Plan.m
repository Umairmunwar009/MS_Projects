function varargout = Robot_Path_Plan(varargin)
% ROBOT_PATH_PLAN MATLAB code for Robot_Path_Plan.fig
%      ROBOT_PATH_PLAN, by itself, creates a new ROBOT_PATH_PLAN or raises the existing
%      singleton*.
%
%      H = ROBOT_PATH_PLAN returns the handle to a new ROBOT_PATH_PLAN or the handle to
%      the existing singleton*.
%
%      ROBOT_PATH_PLAN('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in ROBOT_PATH_PLAN.M with the given input arguments.
%
%      ROBOT_PATH_PLAN('Property','Value',...) creates a new ROBOT_PATH_PLAN or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before Robot_Path_Plan_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to Robot_Path_Plan_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help Robot_Path_Plan

% Last Modified by GUIDE v2.5 15-Jan-2022 15:06:47

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @Robot_Path_Plan_OpeningFcn, ...
                   'gui_OutputFcn',  @Robot_Path_Plan_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before Robot_Path_Plan is made visible.
function Robot_Path_Plan_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to Robot_Path_Plan (see VARARGIN)

% Choose default command line output for Robot_Path_Plan
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes Robot_Path_Plan wait for user response (see UIRESUME)
% uiwait(handles.figure1);
global X;X=[];
global Y;Y=[];


% --- Outputs from this function are returned to the command line.
function varargout = Robot_Path_Plan_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;



function Theta_1_Callback(hObject, eventdata, handles)
% hObject    handle to Theta_1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Theta_1 as text
%        str2double(get(hObject,'String')) returns contents of Theta_1 as a double


% --- Executes during object creation, after setting all properties.
function Theta_1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Theta_1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Theta_2_Callback(hObject, eventdata, handles)
% hObject    handle to Theta_2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Theta_2 as text
%        str2double(get(hObject,'String')) returns contents of Theta_2 as a double


% --- Executes during object creation, after setting all properties.
function Theta_2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Theta_2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Theta_3_Callback(hObject, eventdata, handles)
% hObject    handle to Theta_3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Theta_3 as text
%        str2double(get(hObject,'String')) returns contents of Theta_3 as a double


% --- Executes during object creation, after setting all properties.
function Theta_3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Theta_3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Theta_4_Callback(hObject, eventdata, handles)
% hObject    handle to Theta_4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Theta_4 as text
%        str2double(get(hObject,'String')) returns contents of Theta_4 as a double


% --- Executes during object creation, after setting all properties.
function Theta_4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Theta_4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function Pos_X_Callback(hObject, eventdata, handles)
% hObject    handle to Pos_X (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Pos_X as text
%        str2double(get(hObject,'String')) returns contents of Pos_X as a double


% --- Executes during object creation, after setting all properties.
function Pos_X_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Pos_X (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Pos_Y_Callback(hObject, eventdata, handles)
% hObject    handle to Pos_Y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Pos_Y as text
%        str2double(get(hObject,'String')) returns contents of Pos_Y as a double


% --- Executes during object creation, after setting all properties.
function Pos_Y_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Pos_Y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Pos_Z_Callback(hObject, eventdata, handles)
% hObject    handle to Pos_Z (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Pos_Z as text
%        str2double(get(hObject,'String')) returns contents of Pos_Z as a double


% --- Executes during object creation, after setting all properties.
function Pos_Z_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Pos_Z (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in btn_Forward.
function btn_Forward_Callback(hObject, eventdata, handles)
% hObject    handle to btn_Forward (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    Th_1 =str2double(handles.Theta_1.String)*pi/180;
    Th_2 =str2double(handles.Theta_2.String)*pi/180;
    Th_3 =str2double(handles.Theta_3.String)*pi/180;
    Th_4 =str2double(handles.Theta_4.String)*pi/180;
    Th_5 =str2double(handles.Theta_5.String)*pi/180;
    Th_6 =str2double(handles.Theta_6.String)*pi/180;

    handles.Message.String = num2str(' Please enter value of Link Length above');

    dh_robot
   %Th_1=30;Th_2=45;Th_3=30;Th_4=10;
   rob.plot([Th_1 Th_2 Th_3 Th_4 Th_5 Th_6]);
   T= rob.fkine([Th_1 Th_2 Th_3 Th_4 Th_5 Th_6]);
   Tt=T.t;
   handles.Pos_X.String = num2str((floor(Tt(1))));
   handles.Pos_Y.String = num2str((floor(Tt(2))));
   handles.Pos_Z.String = num2str((floor(Tt(3))));
   
% --- Executes on button press in btn_Inverse.
function btn_Inverse_Callback(hObject, eventdata, handles)
% hObject    handle to btn_Inverse (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
   
dh_robot
PX=str2double(handles.Pos_X.String);PY=str2double(handles.Pos_Y.String);PZ=str2double(handles.Pos_Z.String);start=[PX PY PZ];
T0=[1 0 0 PX;0 1 0 PY;0 0 1 PZ;0 0 0 1];q0=rob.ikine(T0, [0 0 0], 'mask',[1 1 1 0 0 0])*180/pi;
PX=str2double(handles.Pos_X.String);PY=str2double(handles.Pos_Y.String);PZ=str2double(handles.Pos_Z.String);last=[PX PY PZ];via=[start;last];
T= [1 0 0 PX;0 1 0 PY;0 0 1 PZ;0 0 0 1];qf1=rob.ikine(T, [0 0 0], 'mask',[1 1 1 0 0 0])*180/pi;
% ----------------- Ploting Trajectory 1 to 2
x = mstraj(via, [10 10 10], [], start, inter, 10);  % via points, velocity, start point, timestep, acceleration
plot3(x(:,1),x(:,2),x(:,3),'-b','LineWidth',3);grid on;hold on;%plot3(PX1,PY1,PZ1,'or');hold on;plot3(PX2,PY2, PZ2,'ob');

handles.Theta_1.String = num2str((floor(qf1(1))));
handles.Theta_2.String = num2str((floor(qf1(2))));
handles.Theta_3.String = num2str((floor(qf1(3))));
handles.Theta_4.String = num2str((floor(qf1(4))));
   
    temp=0:2:50;q=jtraj(q0,qf1,temp);
    if q ==0
        handles.Message.String = num2str(' The point is singular ');
    else
        handles.Message.String = num2str(' Non Sigular');
    end
    q0=qf1;rob.plot(q*pi/180);

% --- Executes during object creation, after setting all properties.
function uipanel1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to uipanel1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called



function PX2_Callback(hObject, eventdata, handles)
% hObject    handle to PX2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)



% Hints: get(hObject,'String') returns contents of PX2 as text
%        str2double(get(hObject,'String')) returns contents of PX2 as a double


% --- Executes during object creation, after setting all properties.
function PX2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to PX2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in btn_fullstretch.
function btn_fullstretch_Callback(hObject, eventdata, handles)
% hObject    handle to btn_fullstretch (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

dh_robot
   
   Th_1=0;Th_2=0;Th_3=0;Th_4=0;Th_5=0;Th_6=0;
       rob.plot([Th_1 Th_2 Th_3 Th_4 Th_5 Th_6]);
   T= rob.fkine([Th_1 Th_2 Th_3 Th_4 Th_5 Th_6]);
   Tt=T.t;
   handles.Pos_X.String = num2str((floor(Tt(1))));
   handles.Pos_Y.String = num2str((floor(Tt(2))));
   handles.Pos_Z.String = num2str((floor(Tt(3))));

handles.Theta_1.String = num2str((floor(Th_1)));
handles.Theta_2.String = num2str((floor(Th_2)));
handles.Theta_3.String = num2str((floor(Th_3)));
handles.Theta_4.String = num2str((floor(Th_4)));
handles.Theta_5.String = num2str((floor(Th_5)));
handles.Theta_6.String = num2str((floor(Th_6)));
   
  


% --- Executes when uipanel2 is resized.
function uipanel2_SizeChangedFcn(hObject, eventdata, handles)
% hObject    handle to uipanel2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)



function PY1_Callback(hObject, eventdata, handles)
% hObject    handle to PY1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of PY1 as text
%        str2double(get(hObject,'String')) returns contents of PY1 as a double


% --- Executes during object creation, after setting all properties.
function PY1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to PY1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function PZ1_Callback(hObject, eventdata, handles)
% hObject    handle to PZ1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of PZ1 as text
%        str2double(get(hObject,'String')) returns contents of PZ1 as a double


% --- Executes during object creation, after setting all properties.
function PZ1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to PZ1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in btn_set2.
function btn_set2_Callback(hObject, eventdata, handles)
% hObject    handle to btn_set2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
 %% ======  Update Coordinate ========
read_para_dh
PX1=str2double(handles.PX1.String);
PY1=str2double(handles.PY1.String);
PZ1=str2double(handles.PZ1.String);
%% Writing Joint angle
handles.PX1.String = num2str((floor(PX1)));
handles.PY1.String = num2str((floor(PY1)));
handles.PZ1.String = num2str((floor(PZ1)));
check_points(PX1,PY1,PZ1,PX_L,PY_L,PZ_L)


        

% --- Executes on button press in btn_home.
function btn_home_Callback(hObject, eventdata, handles)
% hObject    handle to btn_home (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

   dh_robot
   workspace
   init_goal;                   % load initial & goal coordinates & radius
   PX=P(1);PY=P(2);PZ=P(3);
%    PX=-1000;PY=1500;PZ=0;
   T=[1 0 0 PX;
      0 1 0 PY;
      0 0 1 PZ;
      0 0 0 1];
                                %returns joint angles for given home pose
q=rob.ikine(T, [0 0 0], 'mask',[1 1 1 0 0 0])*180/pi;
% q = R.jtraj(T1, t2, k, options)
% q = R.ikine6s(T)
axes(handles.axes1)
rob.plot(q*pi/180);hold on;

   handles.Theta_1.String = num2str((floor(q(1))));
   handles.Theta_2.String = num2str((floor(q(2))));
   handles.Theta_3.String = num2str((floor(q(3))));
   handles.Theta_4.String = num2str((floor(q(4))));
   handles.Theta_5.String = num2str((floor(q(5))));
   handles.Theta_6.String = num2str((floor(q(6))));
   
   handles.Pos_X.String = num2str((floor(PX)));
   handles.Pos_Y.String = num2str((floor(PY)));
   handles.Pos_Z.String = num2str((floor(PZ)));
   
%     LPx=[P1(1) P2(1) P3(1) P4(1) P1(1)];
%     LPy=[P1(2) P2(2) P3(2) P4(2) P1(2)];
    %LPz=[0 0 0 0]
%     plot( LPx,LPy,'-r');
 
% --- Executes on button press in btn_start.
function btn_start_Callback(hObject, eventdata, handles)
% hObject    handle to btn_start (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

offset_y=3000;curve_para=12;NN=1;iter_smooth = 10000;smooth_per_edit=1.0;
init= [20 20 0 50];goal=[400 400 0 30];
inter=2;
L1=600;  L2=800;  L3=900;  L4=900;  L5=950;  L6=0;  d1=0;d2=0;d3=0;d4=0;d5=0;d6=0;
alpha1=0;alpha2=0;alpha3=0;alpha4=0;alpha5=0;alpha6=0;o1=0;o2=0;o3=0;o4=0;o5=0;o6=0;

%% Writing in boxes
handles.x_initial.String = num2str((floor(init(1))));
handles.y_initial.String = num2str((floor(init(2))));
handles.z_initial.String = num2str((floor(init(3))));
handles.r_init.String    = num2str((floor(init(4))));

handles.x_goal.String     = num2str((floor(goal(1))));
handles.y_goal.String     = num2str((floor(goal(2))));
handles.z_goal.String     = num2str((floor(goal(3))));
handles.goal_rad.String   = num2str((floor(goal(4))));

handles.inter.String = num2str((floor(inter)));
handles.Y_offset.String    = num2str((floor(offset_y)));
handles.curve_para.String    = num2str((floor(curve_para)));
handles.NN.String    = num2str((floor(NN)));

handles.iter_smooth.String = num2str((floor(iter_smooth)));
handles.smooth_per_edit.String = num2str((floor(smooth_per_edit)));
%%
write_para_dh
dh_robot

function Link_1_Callback(hObject, eventdata, handles)
% hObject    handle to Link_1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Link_1 as text
%        str2double(get(hObject,'String')) returns contents of Link_1 as a double


% --- Executes during object creation, after setting all properties.
function Link_1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Link_1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Link_2_Callback(hObject, eventdata, handles)
% hObject    handle to Link_2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Link_2 as text
%        str2double(get(hObject,'String')) returns contents of Link_2 as a double


% --- Executes during object creation, after setting all properties.
function Link_2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Link_2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Link_3_Callback(hObject, eventdata, handles)
% hObject    handle to Link_3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Link_3 as text
%        str2double(get(hObject,'String')) returns contents of Link_3 as a double


% --- Executes during object creation, after setting all properties.
function Link_3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Link_3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Link_4_Callback(hObject, eventdata, handles)
% hObject    handle to Link_4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Link_4 as text
%        str2double(get(hObject,'String')) returns contents of Link_4 as a double


% --- Executes during object creation, after setting all properties.
function Link_4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Link_4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in btn_path_finding.
function btn_path_finding_Callback(hObject, eventdata, handles)
% hObject    handle to btn_path_finding (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% tic
path_finding


% close (h1)




% --- Executes on button press in btn_updatelink.
function btn_updatelink_Callback(hObject, eventdata, handles)
% hObject    handle to btn_updatelink (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


%% ======  Read & Write DH parameters from input ========
%L1=500;L2=500;L3=250;L4=250;
read_para_dh
write_para_dh
dh_robot

workspace
init_goal;                   % load initial & goal coordinates & radius
PX=P(1);PY=P(2);PZ=P(3);

T=[1 0 0 PX;    0 1 0 PY;    0 0 1 PZ;    0 0 0 1];
%returns joint angles for given home pose
q=rob.ikine(T, [0 0 0], 'mask',[1 1 1 0 0 0])*180/pi;
% axes(handles.axes1)
figure (99)
rob.plot(q*pi/180);%hold on;
write_q_p



% --- Executes on button press in btn_Centre_Right.
function btn_Centre_Right_Callback(hObject, eventdata, handles)
% hObject    handle to btn_Centre_Right (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%L1=500;L2=500;L3=250;L4=250;



L1=str2double(handles.Link_1.String);
L2=str2double(handles.Link_2.String);
L3=str2double(handles.Link_3.String);
L4=str2double(handles.Link_4.String);

handles.Link_1.String = num2str((floor(d1)));
handles.Link_2.String = num2str((floor(L2)));
handles.Link_3.String = num2str((floor(L3)));
handles.Link_4.String = num2str((floor(L4)));
dh = [ 0 0 L1 0; 
       0 0 L2 0];
       %0 0 L3 0; 
       %0 0 L4 0 ]
   
   r = SerialLink(dh)
   r.name = 'RRRR_Robot';
   
   PX=0;PY=1000;PZ=500;
   T=[1 0 0 PX;
    0 1 0 PY;
    0 0 1 PZ;
    0 0 0 1]
%returns joint angles for given home pose
q=r.ikine(T, [0 0 0], 'mask',[1 1 1 0 0 0])*180/pi;

handles.Theta_1.String = num2str((floor(q(1))));
handles.Theta_2.String = num2str((floor(q(2))));
handles.Theta_3.String = num2str((floor(q(3))));
handles.Theta_4.String = num2str((floor(q(4))));
   r.plot(q*pi/180);
   handles.Pos_X.String = num2str((floor(PX)));
   handles.Pos_Y.String = num2str((floor(PY)));
   handles.Pos_Z.String = num2str((floor(PZ)));

% --- Executes on button press in btn_Centre_Up.
function btn_Centre_Up_Callback(hObject, eventdata, handles)
% hObject    handle to btn_Centre_Up (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%L1=500;L2=500;L3=250;L4=250;

L1=str2double(handles.Link_1.String);
L2=str2double(handles.Link_2.String);
L3=str2double(handles.Link_3.String);
L4=str2double(handles.Link_4.String);

handles.Link_1.String = num2str((floor(L1)));
handles.Link_2.String = num2str((floor(L2)));
handles.Link_3.String = num2str((floor(L3)));
handles.Link_4.String = num2str((floor(L4)));
dh = [ 0 d1 0 -pi/2; 
       0 0 L2 0;
       0 0 L3 0; 
       0 0 L4 0 ]
   
   r = SerialLink(dh)
   r.name = 'RRRR_Robot';
   
   PX=0;PY=750;PZ=750;
   T=[1 0 0 PX;
    0 1 0 PY;
    0 0 1 PZ;
    0 0 0 1]
%returns joint angles for given home pose
q=r.ikine(T, [0 0 0], 'mask',[1 1 1 0 0 0])*180/pi;

handles.Theta_1.String = num2str((floor(q(1))));
handles.Theta_2.String = num2str((floor(q(2))));
handles.Theta_3.String = num2str((floor(q(3))));
handles.Theta_4.String = num2str((floor(q(4))));
   r.plot(q*pi/180);
   handles.Pos_X.String = num2str((floor(PX)));
   handles.Pos_Y.String = num2str((floor(PY)));
   handles.Pos_Z.String = num2str((floor(PZ)));

% --- Executes on button press in btn_Centre_Left.
function btn_Centre_Left_Callback(hObject, eventdata, handles)
% hObject    handle to btn_Centre_Left (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%L1=500;L2=500;L3=250;L4=250;

d1=str2double(handles.Link_1.String);
L2=str2double(handles.Link_2.String);
L3=str2double(handles.Link_3.String);
L4=str2double(handles.Link_4.String);

handles.Link_1.String = num2str((floor(d1)));
handles.Link_2.String = num2str((floor(L2)));
handles.Link_3.String = num2str((floor(L3)));
handles.Link_4.String = num2str((floor(L4)));
dh = [ 0 d1 0 -pi/2; 
       0 0 L2 0;
       0 0 L3 0; 
       0 0 L4 0 ]
   
   r = SerialLink(dh)
   r.name = 'RRRR_Robot';
   
   PX=0;PY=500;PZ=500;
   T=[1 0 0 PX;
    0 1 0 PY;
    0 0 1 PZ;
    0 0 0 1]
%returns joint angles for given home pose
q=r.ikine(T, [0 0 0], 'mask',[1 1 1 0 0 0])*180/pi;

handles.Theta_1.String = num2str((floor(q(1))));
handles.Theta_2.String = num2str((floor(q(2))));
handles.Theta_3.String = num2str((floor(q(3))));
handles.Theta_4.String = num2str((floor(q(4))));
   r.plot(q*pi/180);
   handles.Pos_X.String = num2str((floor(PX)));
   handles.Pos_Y.String = num2str((floor(PY)));
   handles.Pos_Z.String = num2str((floor(PZ)));

% --- Executes on button press in btn_Centre_Centre.
function btn_Centre_Centre_Callback(hObject, eventdata, handles)
% hObject    handle to btn_Centre_Centre (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%L1=500;L2=500;L3=250;L4=250;

d1=str2double(handles.Link_1.String);
L2=str2double(handles.Link_2.String);
L3=str2double(handles.Link_3.String);
L4=str2double(handles.Link_4.String);

handles.Link_1.String = num2str((floor(d1)));
handles.Link_2.String = num2str((floor(L2)));
handles.Link_3.String = num2str((floor(L3)));
handles.Link_4.String = num2str((floor(L4)));
dh = [ 0 d1 0 -pi/2; 
       0 0 L2 0;
       0 0 L3 0; 
       0 0 L4 0 ]
   
   r = SerialLink(dh)
   r.name = 'RRRR_Robot';
   
   PX=0;PY=750;PZ=500;
   T=[1 0 0 PX;
    0 1 0 PY;
    0 0 1 PZ;
    0 0 0 1]
%returns joint angles for given home pose
q=r.ikine(T, [0 0 0], 'mask',[1 1 1 0 0 0])*180/pi;

handles.Theta_1.String = num2str((floor(q(1))));
handles.Theta_2.String = num2str((floor(q(2))));
handles.Theta_3.String = num2str((floor(q(3))));
handles.Theta_4.String = num2str((floor(q(4))));
   r.plot(q*pi/180);
   handles.Pos_X.String = num2str((floor(PX)));
   handles.Pos_Y.String = num2str((floor(PY)));
   handles.Pos_Z.String = num2str((floor(PZ)));

% --- Executes on button press in btn_Left_Right.
function btn_Left_Right_Callback(hObject, eventdata, handles)
% hObject    handle to btn_Left_Right (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
dh_robot
%    q=[180 0 0 0 0 0];
   Th_1=90;Th_2=0;Th_3=0;Th_4=0;Th_5=0;Th_6=0;
       rob.plot([Th_1 Th_2 Th_3 Th_4 Th_5 Th_6]);
%        rob.plot(q);
       
   T= rob.fkine([Th_1 Th_2 Th_3 Th_4 Th_5 Th_6]);
%    T= rob.fkine(q);
   Tt=T.t;
q(1)=Th_1;q(2)=Th_2;q(3)=Th_3;q(4)=Th_4;q(5)=Th_5;q(6)=Th_6;
PX=Tt(1);PY=Tt(2);PZ=Tt(3);
write_q_p(q,PX,PY,PZ)


% --- Executes on button press in btn_Left_Up.
function btn_Left_Up_Callback(hObject, eventdata, handles)
% hObject    handle to btn_Left_Up (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%L1=500;L2=500;L3=250;L4=250;

d1=str2double(handles.Link_1.String);
L2=str2double(handles.Link_2.String);
L3=str2double(handles.Link_3.String);
L4=str2double(handles.Link_4.String);

handles.Link_1.String = num2str((floor(d1)));
handles.Link_2.String = num2str((floor(L2)));
handles.Link_3.String = num2str((floor(L3)));
handles.Link_4.String = num2str((floor(L4)));
dh = [ 0 d1 0 -pi/2; 
       0 0 L2 0;
       0 0 L3 0; 
       0 0 L4 0 ]
   
   r = SerialLink(dh)
   r.name = 'RRRR_Robot';
   
   PX=-750;PY=0;PZ=750;
   T=[1 0 0 PX;
    0 1 0 PY;
    0 0 1 PZ;
    0 0 0 1]
%returns joint angles for given home pose
q=r.ikine(T, [0 0 0], 'mask',[1 1 1 0 0 0])*180/pi;

handles.Theta_1.String = num2str((floor(q(1))));
handles.Theta_2.String = num2str((floor(q(2))));
handles.Theta_3.String = num2str((floor(q(3))));
handles.Theta_4.String = num2str((floor(q(4))));
   r.plot(q*pi/180);
   handles.Pos_X.String = num2str((floor(PX)));
   handles.Pos_Y.String = num2str((floor(PY)));
   handles.Pos_Z.String = num2str((floor(PZ)));

% --- Executes on button press in btn_Left_Left.
function btn_Left_Left_Callback(hObject, eventdata, handles)
% hObject    handle to btn_Left_Left (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%L1=500;L2=500;L3=250;L4=250;

d1=str2double(handles.Link_1.String);
L2=str2double(handles.Link_2.String);
L3=str2double(handles.Link_3.String);
L4=str2double(handles.Link_4.String);

handles.Link_1.String = num2str((floor(d1)));
handles.Link_2.String = num2str((floor(L2)));
handles.Link_3.String = num2str((floor(L3)));
handles.Link_4.String = num2str((floor(L4)));
dh = [ 0 d1 0 -pi/2; 
       0 0 L2 0;
       0 0 L3 0; 
       0 0 L4 0 ]
   
   r = SerialLink(dh)
   r.name = 'RRRR_Robot';
   
   PX=-1000;PY=0;PZ=500;
   T=[1 0 0 PX;
    0 1 0 PY;
    0 0 1 PZ;
    0 0 0 1]
%returns joint angles for given home pose
q=r.ikine(T, [0 0 0], 'mask',[1 1 1 0 0 0])*180/pi;

handles.Theta_1.String = num2str((floor(q(1))));
handles.Theta_2.String = num2str((floor(q(2))));
handles.Theta_3.String = num2str((floor(q(3))));
handles.Theta_4.String = num2str((floor(q(4))));
   r.plot(q*pi/180);
   handles.Pos_X.String = num2str((floor(PX)));
   handles.Pos_Y.String = num2str((floor(PY)));
   handles.Pos_Z.String = num2str((floor(PZ)));

% --- Executes on button press in btn_Left_Down.
function btn_Left_Down_Callback(hObject, eventdata, handles)
% hObject    handle to btn_Left_Down (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%L1=500;L2=500;L3=250;L4=250;

d1=str2double(handles.Link_1.String);
L2=str2double(handles.Link_2.String);
L3=str2double(handles.Link_3.String);
L4=str2double(handles.Link_4.String);

handles.Link_1.String = num2str((floor(d1)));
handles.Link_2.String = num2str((floor(L2)));
handles.Link_3.String = num2str((floor(L3)));
handles.Link_4.String = num2str((floor(L4)));
dh = [ 0 d1 0 -pi/2; 
       0 0 L2 0;
       0 0 L3 0; 
       0 0 L4 0 ]
   
   r = SerialLink(dh)
   r.name = 'RRRR_Robot';
   
   PX=-750;PY=0;PZ=250;
   T=[1 0 0 PX;
    0 1 0 PY;
    0 0 1 PZ;
    0 0 0 1]
%returns joint angles for given home pose
q=r.ikine(T, [0 0 0], 'mask',[1 1 1 0 0 0])*180/pi;

handles.Theta_1.String = num2str((floor(q(1))));
handles.Theta_2.String = num2str((floor(q(2))));
handles.Theta_3.String = num2str((floor(q(3))));
handles.Theta_4.String = num2str((floor(q(4))));
   r.plot(q*pi/180);
   handles.Pos_X.String = num2str((floor(PX)));
   handles.Pos_Y.String = num2str((floor(PY)));
   handles.Pos_Z.String = num2str((floor(PZ)));

% --- Executes on button press in btn_Right_Right.
function btn_Right_Right_Callback(hObject, eventdata, handles)
% hObject    handle to btn_Right_Right (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%L1=500;L2=500;L3=250;L4=250;

d1=str2double(handles.Link_1.String);
L2=str2double(handles.Link_2.String);
L3=str2double(handles.Link_3.String);
L4=str2double(handles.Link_4.String);

handles.Link_1.String = num2str((floor(d1)));
handles.Link_2.String = num2str((floor(L2)));
handles.Link_3.String = num2str((floor(L3)));
handles.Link_4.String = num2str((floor(L4)));
dh = [ 0 d1 0 -pi/2; 
       0 0 L2 0;
       0 0 L3 0; 
       0 0 L4 0 ]
   
   r = SerialLink(dh)
   r.name = 'RRRR_Robot';
   
   PX=1000;PY=0;PZ=500;
   T=[1 0 0 PX;
    0 1 0 PY;
    0 0 1 PZ;
    0 0 0 1]
%returns joint angles for given home pose
q=r.ikine(T, [0 0 0], 'mask',[1 1 1 0 0 0])*180/pi;

handles.Theta_1.String = num2str((floor(q(1))));
handles.Theta_2.String = num2str((floor(q(2))));
handles.Theta_3.String = num2str((floor(q(3))));
handles.Theta_4.String = num2str((floor(q(4))));
   r.plot(q*pi/180);
   handles.Pos_X.String = num2str((floor(PX)));
   handles.Pos_Y.String = num2str((floor(PY)));
   handles.Pos_Z.String = num2str((floor(PZ)));


% --- Executes on button press in btn_Up_Right.
function btn_Up_Right_Callback(hObject, eventdata, handles)
% hObject    handle to btn_Up_Right (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%L1=500;L2=500;L3=250;L4=250;

d1=str2double(handles.Link_1.String);
L2=str2double(handles.Link_2.String);
L3=str2double(handles.Link_3.String);
L4=str2double(handles.Link_4.String);

handles.Link_1.String = num2str((floor(d1)));
handles.Link_2.String = num2str((floor(L2)));
handles.Link_3.String = num2str((floor(L3)));
handles.Link_4.String = num2str((floor(L4)));
dh = [ 0 d1 0 -pi/2; 
       0 0 L2 0;
       0 0 L3 0; 
       0 0 L4 0 ]
   
   r = SerialLink(dh)
   r.name = 'RRRR_Robot';
   
   PX=750;PY=0;PZ=750;
   T=[1 0 0 PX;
    0 1 0 PY;
    0 0 1 PZ;
    0 0 0 1]
%returns joint angles for given home pose
q=r.ikine(T, [0 0 0], 'mask',[1 1 1 0 0 0])*180/pi;

handles.Theta_1.String = num2str((floor(q(1))));
handles.Theta_2.String = num2str((floor(q(2))));
handles.Theta_3.String = num2str((floor(q(3))));
handles.Theta_4.String = num2str((floor(q(4))));
   r.plot(q*pi/180);
   handles.Pos_X.String = num2str((floor(PX)));
   handles.Pos_Y.String = num2str((floor(PY)));
   handles.Pos_Z.String = num2str((floor(PZ)));


% --- Executes on button press in btn_Right_Left.
function btn_Right_Left_Callback(hObject, eventdata, handles)
% hObject    handle to btn_Right_Left (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%L1=500;L2=500;L3=250;L4=250;

d1=str2double(handles.Link_1.String);
L2=str2double(handles.Link_2.String);
L3=str2double(handles.Link_3.String);
L4=str2double(handles.Link_4.String);

handles.Link_1.String = num2str((floor(d1)));
handles.Link_2.String = num2str((floor(L2)));
handles.Link_3.String = num2str((floor(L3)));
handles.Link_4.String = num2str((floor(L4)));
dh = [ 0 d1 0 -pi/2; 
       0 0 L2 0;
       0 0 L3 0; 
       0 0 L4 0 ]
   
   r = SerialLink(dh)
   r.name = 'RRRR_Robot';
   
   PX=500;PY=0;PZ=500;
   T=[1 0 0 PX;
    0 1 0 PY;
    0 0 1 PZ;
    0 0 0 1]
%returns joint angles for given home pose
q=r.ikine(T, [0 0 0], 'mask',[1 1 1 0 0 0])*180/pi;

handles.Theta_1.String = num2str((floor(q(1))));
handles.Theta_2.String = num2str((floor(q(2))));
handles.Theta_3.String = num2str((floor(q(3))));
handles.Theta_4.String = num2str((floor(q(4))));
   r.plot(q*pi/180);
   handles.Pos_X.String = num2str((floor(PX)));
   handles.Pos_Y.String = num2str((floor(PY)));
   handles.Pos_Z.String = num2str((floor(PZ)));


% --- Executes on button press in btn_right_down.
function btn_right_down_Callback(hObject, eventdata, handles)
% hObject    handle to btn_right_down (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%L1=500;L2=500;L3=250;L4=250;

d1=str2double(handles.Link_1.String);
L2=str2double(handles.Link_2.String);
L3=str2double(handles.Link_3.String);
L4=str2double(handles.Link_4.String);

handles.Link_1.String = num2str((floor(d1)));
handles.Link_2.String = num2str((floor(L2)));
handles.Link_3.String = num2str((floor(L3)));
handles.Link_4.String = num2str((floor(L4)));
dh = [ 0 d1 0 -pi/2; 
       0 0 L2 0;
       0 0 L3 0; 
       0 0 L4 0 ]
   
   r = SerialLink(dh)
   r.name = 'RRRR_Robot';
   
   PX=750;PY=0;PZ=250;
   T=[1 0 0 PX;
    0 1 0 PY;
    0 0 1 PZ;
    0 0 0 1]
%returns joint angles for given home pose
q=r.ikine(T, [0 0 0], 'mask',[1 1 1 0 0 0])*180/pi;

handles.Theta_1.String = num2str((floor(q(1))));
handles.Theta_2.String = num2str((floor(q(2))));
handles.Theta_3.String = num2str((floor(q(3))));
handles.Theta_4.String = num2str((floor(q(4))));
   r.plot(q*pi/180);
   handles.Pos_X.String = num2str((floor(PX)));
   handles.Pos_Y.String = num2str((floor(PY)));
   handles.Pos_Z.String = num2str((floor(PZ)));


% --- Executes on button press in btn_Right_centre.
function btn_Right_centre_Callback(hObject, eventdata, handles)
% hObject    handle to btn_Right_centre (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%L1=500;L2=500;L3=250;L4=250;

d1=str2double(handles.Link_1.String);
L2=str2double(handles.Link_2.String);
L3=str2double(handles.Link_3.String);
L4=str2double(handles.Link_4.String);

handles.Link_1.String = num2str((floor(d1)));
handles.Link_2.String = num2str((floor(L2)));
handles.Link_3.String = num2str((floor(L3)));
handles.Link_4.String = num2str((floor(L4)));
dh = [ 0 d1 0 -pi/2; 
       0 0 L2 0;
       0 0 L3 0; 
       0 0 L4 0 ]
   
   r = SerialLink(dh)
   r.name = 'RRRR_Robot';
   
   PX=750;PY=0;PZ=500;
   T=[1 0 0 PX;
    0 1 0 PY;
    0 0 1 PZ;
    0 0 0 1]
%returns joint angles for given home pose
q=r.ikine(T, [0 0 0], 'mask',[1 1 1 0 0 0])*180/pi;

handles.Theta_1.String = num2str((floor(q(1))));
handles.Theta_2.String = num2str((floor(q(2))));
handles.Theta_3.String = num2str((floor(q(3))));
handles.Theta_4.String = num2str((floor(q(4))));
   r.plot(q*pi/180);
   handles.Pos_X.String = num2str((floor(PX)));
   handles.Pos_Y.String = num2str((floor(PY)));
   handles.Pos_Z.String = num2str((floor(PZ)));


% --- Executes on button press in btn_Left_Center.
function btn_Left_Center_Callback(hObject, eventdata, handles)
% hObject    handle to btn_Left_Center (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%L1=500;L2=500;L3=250;L4=250;

d1=str2double(handles.Link_1.String);
L2=str2double(handles.Link_2.String);
L3=str2double(handles.Link_3.String);
L4=str2double(handles.Link_4.String);

handles.Link_1.String = num2str((floor(d1)));
handles.Link_2.String = num2str((floor(L2)));
handles.Link_3.String = num2str((floor(L3)));
handles.Link_4.String = num2str((floor(L4)));
dh = [ 0 d1 0 -pi/2; 
       0 0 L2 0;
       0 0 L3 0; 
       0 0 L4 0 ]
   
   r = SerialLink(dh)
   r.name = 'RRRR_Robot';
   
   PX=-750;PY=0;PZ=500;
   T=[1 0 0 PX;
    0 1 0 PY;
    0 0 1 PZ;
    0 0 0 1]
%returns joint angles for given home pose
q=r.ikine(T, [0 0 0], 'mask',[1 1 1 0 0 0])*180/pi;

handles.Theta_1.String = num2str((floor(q(1))));
handles.Theta_2.String = num2str((floor(q(2))));
handles.Theta_3.String = num2str((floor(q(3))));
handles.Theta_4.String = num2str((floor(q(4))));
   r.plot(q*pi/180);
   handles.Pos_X.String = num2str((floor(PX)));
   handles.Pos_Y.String = num2str((floor(PY)));
   handles.Pos_Z.String = num2str((floor(PZ)));

% --- Executes on button press in btn_Centre_Centre.
function pushbutton13_Callback(hObject, eventdata, handles)
% hObject    handle to btn_Centre_Centre (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in btn_centre_down.
function btn_centre_down_Callback(hObject, eventdata, handles)
% hObject    handle to btn_centre_down (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%L1=500;L2=500;L3=250;L4=250;

d1=str2double(handles.Link_1.String);
L2=str2double(handles.Link_2.String);
L3=str2double(handles.Link_3.String);
L4=str2double(handles.Link_4.String);

handles.Link_1.String = num2str((floor(d1)));
handles.Link_2.String = num2str((floor(L2)));
handles.Link_3.String = num2str((floor(L3)));
handles.Link_4.String = num2str((floor(L4)));
dh = [ 0 d1 0 -pi/2; 
       0 0 L2 0;
       0 0 L3 0; 
       0 0 L4 0 ]
   
   r = SerialLink(dh)
   r.name = 'RRRR_Robot';
   
   PX=0;PY=750;PZ=250;
   T=[1 0 0 PX;
    0 1 0 PY;
    0 0 1 PZ;
    0 0 0 1]
%returns joint angles for given home pose
q=r.ikine(T, [0 0 0], 'mask',[1 1 1 0 0 0])*180/pi;

handles.Theta_1.String = num2str((floor(q(1))));
handles.Theta_2.String = num2str((floor(q(2))));
handles.Theta_3.String = num2str((floor(q(3))));
handles.Theta_4.String = num2str((floor(q(4))));
   r.plot(q*pi/180);
   handles.Pos_X.String = num2str((floor(PX)));
   handles.Pos_Y.String = num2str((floor(PY)));
   handles.Pos_Z.String = num2str((floor(PZ)));


% --- Executes on button press in btn_robot_cnfig.
function btn_robot_cnfig_Callback(hObject, eventdata, handles)
% hObject    handle to btn_robot_cnfig (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
 %% ======  Read & Write DH parameters from input ========
dh_robot
% load_points
workspace
inter=str2double(handles.inter.String);
PX=P(1);PX1=P1(1);PX2=P2(1);PX3=P3(1);PX4=P4(1);PX5=P(1);
PY=P(2);PY1=P1(2);PY2=P2(2);PY3=P3(2);PY4=P4(2);PY5=P(2);
PZ=P(3);PZ1=P1(3);PZ2=P2(3);PZ3=P3(3);PZ4=P4(3);PZ5=P(3);
% check_points
%% ===============   Transformation Matrix & Inverse Kinematics =========================
T0=[1 0 0 PX;0 1 0 PY;0 0 1 PZ;0 0 0 1];   q0=rob.ikine(T0, [0 0 0], 'mask',[1 1 1 0 0 0]);%*180/pi;
T1=[1 0 0 PX1;0 1 0 PY1;0 0 1 PZ1;0 0 0 1];q1=rob.ikine(T1, [0 0 0], 'mask',[1 1 1 0 0 0]);%*180/pi;
T2=[1 0 0 PX2;0 1 0 PY2;0 0 1 PZ2;0 0 0 1];q2=rob.ikine(T2, [0 0 0], 'mask',[1 1 1 0 0 0]);%*180/pi;
T3=[1 0 0 PX3;0 1 0 PY3;0 0 1 PZ3;0 0 0 1];q3=rob.ikine(T3, [0 0 0], 'mask',[1 1 1 0 0 0]);%*180/pi;
T4=[1 0 0 PX4;0 1 0 PY4;0 0 1 PZ4;0 0 0 1];q4=rob.ikine(T4, [0 0 0], 'mask',[1 1 1 0 0 0]);%*180/pi;
T5=[1 0 0 PX5;0 1 0 PY5;0 0 1 PZ5;0 0 0 1];q5=rob.ikine(T5, [0 0 0], 'mask',[1 1 1 0 0 0]);%*180/pi;

joint_correction
%% ====================    Robot Move & Trajectory of all Points ==================
% Robot move & Ploting Trajectory 0 to 1
start=[PX PY PZ];last=[PX1 PY1 PZ1];via=[start;last];
x = mstraj(via, [10 10 10], [], start, inter, 10);  % via points, velocity, start point, timestep, acceleration
plot3(x(:,1),x(:,2),x(:,3),'-g','LineWidth',3);grid on;hold on;plot3(PX,PY,PZ,'or');hold on;plot3(PX1,PY1, PZ1,'ob');
T=ctraj(T0, T1, inter);qpp=rob.ikine(T, [0 0 0], 'mask',[1 1 1 0 0 0]);rob.plot(qpp);
write_q_p(q1,PX1,PY1,PZ1)
% -----------------Robot move & Ploting Trajectory 1 to 2
start=[PX1 PY1 PZ1];last=[PX2 PY2 PZ2];via=[start;last];
x = mstraj(via, [10 10 10], [], start, inter, 10);  % via points, velocity, start point, timestep, acceleration
plot3(x(:,1),x(:,2),x(:,3),'-b','LineWidth',3);grid on;hold on;plot3(PX1,PY1,PZ1,'or');hold on;plot3(PX2,PY2, PZ2,'ob');
T=ctraj(T1, T2, inter);qpp=rob.ikine(T, [0 0 0], 'mask',[1 1 1 0 0 0]);rob.plot(qpp);
% -----------------Robot move & Ploting Trajectory 2 to 3
start=[PX2 PY2 PZ2];last=[PX3 PY3 PZ3];via=[start;last];
x = mstraj(via, [10 10 10], [], start, inter, 10);  % via points, velocity, start point, timestep, acceleration
plot3(x(:,1),x(:,2),x(:,3),'-k','LineWidth',3);grid on;hold on;plot3(PX2,PY2,PZ2,'or');hold on;plot3(PX3,PY3, PZ3,'ob');
T=ctraj(T2, T3, inter);qpp=rob.ikine(T, [0 0 0], 'mask',[1 1 1 0 0 0]);rob.plot(qpp);
% -----------------Robot move & Ploting Trajectory 3 to 4
start=[PX3 PY3 PZ3];last=[PX4 PY4 PZ4];via=[start;last];
x = mstraj(via, [10 10 10], [], start, inter, 10);  % via points, velocity, start point, timestep, acceleration
plot3(x(:,1),x(:,2),x(:,3),'-r','LineWidth',3);grid on;hold on;plot3(PX3,PY3,PZ3,'or');hold on;plot3(PX4,PY4, PZ4,'ob');
T=ctraj(T3, T4, inter);qpp=rob.ikine(T, [0 0 0], 'mask',[1 1 1 0 0 0]);rob.plot(qpp);
% -----------------Robot move & Ploting Trajectory 4 to 1
start=[PX4 PY4 PZ4];last=[PX1 PY1 PZ1];via=[start;last];
x = mstraj(via, [10 10 10], [], start, inter, 10);  % via points, velocity, start point, timestep, acceleration
plot3(x(:,1),x(:,2),x(:,3),'-c','LineWidth',3);grid on;hold on;plot3(PX4,PY4,PZ4,'or');hold on;plot3(PX5,PY5, PZ5,'ob');
T=ctraj(T4, T1, inter);qpp=rob.ikine(T, [0 0 0], 'mask',[1 1 1 0 0 0]);rob.plot(qpp);
% -----------------Robot move & Ploting Trajectory 1 to 5
start=[PX1 PY1 PZ1];last=[PX PY PZ];via=[start;last];
x = mstraj(via, [10 10 10], [], start, inter, 10);  % via points, velocity, start point, timestep, acceleration
plot3(x(:,1),x(:,2),x(:,3),'-m','LineWidth',3);grid on;hold on;plot3(PX5,PY5,PZ5,'or');hold on;plot3(PX1,PY1, PZ1,'ob');
T=ctraj(T1, T0, inter);qpp=rob.ikine(T, [0 0 0], 'mask',[1 1 1 0 0 0]);rob.plot(qpp);
%% Writing Joint angle
handles.Theta_1.String = num2str((floor(q1(1)*180/pi)));
handles.Theta_2.String = num2str((floor(q1(2)*180/pi)));
handles.Theta_3.String = num2str((floor(q1(3)*180/pi)));
handles.Theta_4.String = num2str((floor(q1(4)*180/pi)));
%handles.out.String = num2str((floor(out)));
handles.Theta_1.String = num2str((floor(q2(1)*180/pi)));
handles.Theta_2.String = num2str((floor(q2(2)*180/pi)));
handles.Theta_3.String = num2str((floor(q2(3)*180/pi)));
handles.Theta_4.String = num2str((floor(q2(4)*180/pi)));
%handles.out.String = num2str((floor(out)));
handles.Theta_1.String = num2str((floor(q3(1)*180/pi)));
handles.Theta_2.String = num2str((floor(q3(2)*180/pi)));
handles.Theta_3.String = num2str((floor(q3(3)*180/pi)));
handles.Theta_4.String = num2str((floor(q3(4)*180/pi)));
% handles.out.String = num2str((floor(out)));
handles.Theta_1.String = num2str((floor(q4(1)*180/pi)));
handles.Theta_2.String = num2str((floor(q4(2)*180/pi)));
handles.Theta_3.String = num2str((floor(q4(3)*180/pi)));
handles.Theta_4.String = num2str((floor(q4(4)*180/pi)));
% handles.out.String = num2str((floor(out)));
handles.Theta_1.String = num2str((floor(q5(1)*180/pi)));
handles.Theta_2.String = num2str((floor(q5(2)*180/pi)));
handles.Theta_3.String = num2str((floor(q5(3)*180/pi)));
handles.Theta_4.String = num2str((floor(q5(4)*180/pi)));
% handles.out.String = num2str((floor(out)));

function PX3_Callback(hObject, eventdata, handles)
% hObject    handle to PX3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of PX3 as text
%        str2double(get(hObject,'String')) returns contents of PX3 as a double


% --- Executes during object creation, after setting all properties.
function PX3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to PX3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function PY3_Callback(hObject, eventdata, handles)
% hObject    handle to PY3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of PY3 as text
%        str2double(get(hObject,'String')) returns contents of PY3 as a double


% --- Executes during object creation, after setting all properties.
function PY3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to PY3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function PZ3_Callback(hObject, eventdata, handles)
% hObject    handle to PZ3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of PZ3 as text
%        str2double(get(hObject,'String')) returns contents of PZ3 as a double


% --- Executes during object creation, after setting all properties.
function PZ3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to PZ3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function PX4_Callback(hObject, eventdata, handles)
% hObject    handle to PX4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of PX4 as text
%        str2double(get(hObject,'String')) returns contents of PX4 as a double


% --- Executes during object creation, after setting all properties.
function PX4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to PX4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function PY4_Callback(hObject, eventdata, handles)
% hObject    handle to PY4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of PY4 as text
%        str2double(get(hObject,'String')) returns contents of PY4 as a double


% --- Executes during object creation, after setting all properties.
function PY4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to PY4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function PZ4_Callback(hObject, eventdata, handles)
% hObject    handle to PZ4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of PZ4 as text
%        str2double(get(hObject,'String')) returns contents of PZ4 as a double


% --- Executes during object creation, after setting all properties.
function PZ4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to PZ4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function PX5_Callback(hObject, eventdata, handles)
% hObject    handle to PX5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of PX5 as text
%        str2double(get(hObject,'String')) returns contents of PX5 as a double


% --- Executes during object creation, after setting all properties.
function PX5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to PX5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function PY5_Callback(hObject, eventdata, handles)
% hObject    handle to PY5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of PY5 as text
%        str2double(get(hObject,'String')) returns contents of PY5 as a double


% --- Executes during object creation, after setting all properties.
function PY5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to PY5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function PZ5_Callback(hObject, eventdata, handles)
% hObject    handle to PZ5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of PZ5 as text
%        str2double(get(hObject,'String')) returns contents of PZ5 as a double


% --- Executes during object creation, after setting all properties.
function PZ5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to PZ5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in btn_set5.
function btn_set5_Callback(hObject, eventdata, handles)
% hObject    handle to btn_set5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

%% ======  Update Coordinate ========
PX5=str2double(handles.PX5.String);
PY5=str2double(handles.PY5.String);
PZ5=str2double(handles.PZ5.String);
%% =======  Writing    =============
handles.PX5.String = num2str((floor(PX5)));
handles.PY5.String = num2str((floor(PY5)));
handles.PZ5.String = num2str((floor(PZ5)));
read_para_dh
check_points(PX5,PY5,PZ5,PX_L,PY_L,PZ_L)

% --- Executes on button press in btn_set4.
function btn_set4_Callback(hObject, eventdata, handles)
% hObject    handle to btn_set4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

%% ======  Update Coordinate ========
PX4=str2double(handles.PX4.String);
PY4=str2double(handles.PY4.String);
PZ4=str2double(handles.PZ4.String);
%% =======  Writing    =============
handles.PX4.String = num2str((floor(PX4)));
handles.PY4.String = num2str((floor(PY4)));
handles.PZ4.String = num2str((floor(PZ4)));
%% Valid Points
read_para_dh
check_points(PX4,PY4,PZ4,PX_L,PY_L,PZ_L)


%% ========== Velocity Ploting =========
% [q ,qd ,qdd] =jtraj(q3,q4,inter, [0 0 0 0], [10 10 10 10]);
% figure(34);title('Joint Angles  Velocities Accelerations from Point 2 to Point 3');
%         subplot(3,1,1);plot(q);title('Joint Angle vs. time');xlabel('Time(s)');ylabel('Joint Angle (deg)');legend('q1','q2','q3','q4');
%         subplot(3,1,2);plot(qd);title('Joint velocities vs. time');xlabel('Time(s)');ylabel('Joint Velocities (deg/s)') 
%         subplot(3,1,3);plot(qdd);title('Joint Accelration vs. time');xlabel('Time(s)');ylabel('Joint Acceleration (deg/s^2)') 

% --- Executes on button press in btn_set3.
function btn_set3_Callback(hObject, eventdata, handles)
% hObject    handle to btn_set3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

%% ======  Update Coordinate ========
read_para_dh
PX3=str2double(handles.PX3.String);
PY3=str2double(handles.PY3.String);
PZ3=str2double(handles.PZ3.String);
%% Writing Joint angle
handles.PX3.String = num2str((floor(PX3)));
handles.PY3.String = num2str((floor(PY3)));
handles.PZ3.String = num2str((floor(PZ3)));
check_points(PX3,PY3,PZ3,PX_L,PY_L,PZ_L)

function PX1_Callback(hObject, eventdata, handles)
% hObject    handle to PX1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of PX1 as text
%        str2double(get(hObject,'String')) returns contents of PX1 as a double


% --- Executes during object creation, after setting all properties.
function PX1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to PX1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in btn_Set1.
function btn_Set1_Callback(hObject, eventdata, handles)
% hObject    handle to btn_Set1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
 
%% ======  Update Coordinate ========
read_para_dh
PX1=str2double(handles.PX1.String);
PY1=str2double(handles.PY1.String);
PZ1=str2double(handles.PZ1.String);
%% Writing Joint angle
handles.PX1.String = num2str((floor(PX1)));
handles.PY1.String = num2str((floor(PY1)));
handles.PZ1.String = num2str((floor(PZ1)));
check_points(PX1,PY1,PZ1,PX_L,PY_L,PZ_L)


% --- Executes on button press in btn_close.
function btn_close_Callback(hObject, eventdata, handles)
% hObject    handle to btn_close (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
close all;
clc;
clear all;

% --- Executes on button press in btn_load_traj.
function btn_load_traj_Callback(hObject, eventdata, handles)
% hObject    handle to btn_load_traj (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

offset_y=str2double(handles.Y_offset.String); % read y offset from panel
global X;
global Y;
x=X;y=Y;
vx=10000;vy=10000;vz=10000;
%  load('path.mat')            % load Only variable path_node
%% for loading fiited path
% load('path_fit.mat')         % load curve fit path_node
% y=ft_path;
%%
init_goal

init_c =[init(1) init(2)+offset_y  init(3)  init(4)];
goal_c =[goal(1) goal(2)+offset_y  goal(3)  goal(4)];
a = get(handles.obstacle_selection,'Value');
[Obstacle] = obstacle(a);
Obstacle_c = [Obstacle(:,1)  Obstacle(:,2)+offset_y   Obstacle(:,3)];
% figure (100);
axes(handles.axes1)
axis([-2000 2000 -500 4050]);
% axis tight
rectangle('Position',[-1000 -1000+offset_y 2000 2000],'EdgeColor','b','LineWidth',2);
viscircles(Obstacle_c(:,1:2),Obstacle_c(:,3));
circle(init_c,r_init);circle(goal_c,goal_rad);

% curve_para=
% [fitresult, gof] = createFit(xxx, yyy, curve_para)

plot(x,y+offset_y,'-m','LineWidth',2);

PX = init(1);PY = init(2)+offset_y; PZ=0;
dh_robot
tic
for i= 1:1:length(x)
    if i == length(x)
        PX1 = goal(1);PY1 = goal(2)+offset_y; PZ1=0;
        handles.Message.String = num2str('..... GOAL ACHIEVED!!!! ...........');
    else
        PX1 = x(i);PY1 = y(i)+offset_y; PZ1=0;
        handles.Message.String = num2str('..... Robot Motion on trajectory in process ...........');
    end
%% ===============   Transformation Matrix & Inverse Kinematics =========================
T0=[1 0 0 PX;0 1 0 PY;0 0 1 PZ;0 0 0 1];   q0=rob.ikine(T0, [0 0 0], 'mask',[1 1 1 0 0 0])*180/pi;
T1=[1 0 0 PX1;0 1 0 PY1;0 0 1 PZ1;0 0 0 1];q1=rob.ikine(T1, [0 0 0], 'mask',[1 1 1 0 0 0])*180/pi;

% ====================    Robot Move & Trajectory of all Points ==================
% Robot move & Ploting Trajectory 0 to 1
start=[PX PY PZ];last=[PX1 PY1 PZ1];via=[start;last];
xj = mstraj(via, [vx vy vz], [], start, inter, 1000);  % via points, velocity, start point, timestep, acceleration
plot3(xj(:,1),xj(:,2),xj(:,3),'-c','LineWidth',3);grid on;hold on;plot3(PX,PY,PZ,'-r');hold on;plot3(PX1,PY1, PZ1,'-b');
%% Robot move 
T=ctraj(T0, T1, inter);
qpp=rob.ikine(T, [0 0 0], 'mask',[1 1 1 0 0 0]);
q1=qpp;
% joint_correction
rob.plot(qpp);
%% Writing on GUI

   handles.Theta_1.String = num2str((floor(q1(1)*180/pi)));
   handles.Theta_2.String = num2str((floor(q1(2)*180/pi)));
   handles.Theta_3.String = num2str((floor(q1(3)*180/pi)));
   handles.Theta_4.String = num2str((floor(q1(4)*180/pi)));
   handles.Theta_5.String = num2str((floor(q1(5)*180/pi)));
   handles.Theta_6.String = num2str((floor(q1(6)*180/pi)));
   
   handles.Pos_X.String = num2str((floor(PX1)));
   handles.Pos_Y.String = num2str((floor(PY1)));
   handles.Pos_Z.String = num2str((floor(PZ1)));
   
   PX = PX1; PY = PY1; PZ=0;

end
h2 = msgbox({'Total Time (sec) to reach !!!!   GOAL !!!.........' '' num2str(toc)} ,'Info');
handles.elapsed_time.String = num2str((floor(toc)));
uiwait(h2,30)
% close (h2)



% --- Executes on button press in btn_velocity.
function btn_velocity_Callback(hObject, eventdata, handles)
% hObject    handle to btn_velocity (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
PX=str2double(handles.PX2.String);
PY=str2double(handles.PY1.String);
PZ=str2double(handles.PZ1.String);

%L1=500;L2=500;L3=500;L4=500;
d1=str2double(handles.Link_1.String);
L2=str2double(handles.Link_2.String);
L3=str2double(handles.Link_3.String);
L4=str2double(handles.Link_4.String);

handles.Link_1.String = num2str((floor(d1)));
handles.Link_2.String = num2str((floor(L2)));
handles.Link_3.String = num2str((floor(L3)));
handles.Link_4.String = num2str((floor(L4)));

dh = [ 0 d1 0 -pi/2; 
       0 0 L2 0;
       0 0 L3 0; 
       0 0 L4 0 ]
   
   r = SerialLink(dh)
   r.name = 'RRRR_Robot';
      
   T2=[1 0 0 PX;
    0 1 0 PY;
    0 0 1 PZ;
    0 0 0 1]
%returns joint angles for given home pose
qf2=r.ikine(T2, [0 0 0], 'mask',[1 1 1 0 0 0])*180/pi;

handles.Theta_1.String = num2str((floor(qf2(1))));
handles.Theta_2.String = num2str((floor(qf2(2))));
handles.Theta_3.String = num2str((floor(qf2(3))));
handles.Theta_4.String = num2str((floor(qf2(4))));
    q0=qf2;
    temp=0:1:50;
    q=jtraj(q0,qf2,temp);  
    r.plot(q*pi/180);
   [Q,QD,QDD] = jtraj(q0,qf2, temp, [0 0 0 0], [10 10 10 10]);% initial and final joint velocity for the trajectory and a time vector.
        figure(10);title('Joint Angles Velocities Accelerations')
        subplot(2,2,1);plot(Q*pi/180);title('Joint Angle vs. time');xlabel('Time(s)');ylabel('Joint Angle (deg/s)')  
        subplot(2,2,2);plot(QD);title('Joint Velocites vs. time');xlabel('Time(s)');ylabel('Joint Velocity (mm/s)')
        subplot(2,2,3);plot(QDD);title('Joint Acceleration vs. time');xlabel('Time(s)');ylabel('Joint Acceleration (mm/s^2)')


% --- Executes on button press in btn_trajectory.
function btn_trajectory_Callback(hObject, eventdata, handles)
% hObject    handle to btn_trajectory (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
 %% ======  Read DH parameters from input ========
%L1=500;L2=500;L3=250;L4=250;
    d1=str2double(handles.Link_1.String);
    L2=str2double(handles.Link_2.String);
    L3=str2double(handles.Link_3.String);
    L4=str2double(handles.Link_4.String);
    %% ====== Write DH Parameters ======
handles.Link_1.String = num2str((floor(d1)));
handles.Link_2.String = num2str((floor(L2)));
handles.Link_3.String = num2str((floor(L3)));
handles.Link_4.String = num2str((floor(L4)));
dh = [ 0 d1 0 -pi/2;0 0 L2 0;0 0 L3 0;0 0 L4 0 ]
   r = SerialLink(dh);r.name = 'RRRR_Robot';
   temp=0:2:50; st=50;
%% Create workspace for half donut
        u=linspace(0,pi,100);
        v=linspace(0,2*pi,100);
        [U,V]=meshgrid(u,v);
        X=(3+cos(V))*cos(U)/400*1000;
        Y=(3+cos(V))*sin(U)/400*1000;
        Z=(sin(V)/4+0.5)*1000;
        %creates a semitransparent surface with no edges drawn.
        figure (30);surf(X,Y,Z,'FaceAlpha',0.5,'EdgeColor','none');grid on;hold on; 
   %% ===============  Start Point 1 =========================
PX=str2double(handles.Pos_X.String);
PY=str2double(handles.Pos_Y.String);
PZ=str2double(handles.Pos_Z.String);
% inverse kinematics   
T0=[1 0 0 PX;0 1 0 PY;0 0 1 PZ;0 0 0 1];
q0=r.ikine(T0, [0 0 0], 'mask',[1 1 1 0 0 0])*180/pi
%% ===============  Way Point 1 =========================
PX1=str2double(handles.PX1.String);
PY1=str2double(handles.PY1.String);
PZ1=str2double(handles.PZ1.String);
% inverse kinematics   
T1=[1 0 0 PX1;0 1 0 PY1;0 0 1 PZ1;0 0 0 1];
q1=r.ikine(T1, [0 0 0], 'mask',[1 1 1 0 0 0])*180/pi
R1=sqrt(PX^2 +PY^2);

T=ctraj(T0, T1, st);
Q=r.ikine(T, [0 0 0], 'mask',[1 1 1 0 0 0])*180/pi

figure(30);plot3(x,y,z,'-r');tranimate(T);
%% ================  Way Point 2 =========================
PX2=str2double(handles.PX2.String)
PY2=str2double(handles.PY2.String)
PZ2=str2double(handles.PZ2.String)
% inverse kinematics   
T2=[1 0 0 PX2;0 1 0 PY2;0 0 1 PZ2;0 0 0 1];
q2=r.ikine(T2, [0 0 0], 'mask',[1 1 1 0 0 0])*180/pi

% Trajectery Generarion to rotate q1 only from 1st point to 2nd point
q1_0=[q2(1) q1(2) q1(3) q1(4)];

R1=sqrt(PX1^2 +PY1^2);
int=1;qstep=q1(1):int:q1_0(1);
x=R1*cos(qstep*pi/180);y=R1*sin(qstep*pi/180);
for i=1:length(x)
    z(i)=PZ1;
end
figure(30);plot3(x,y,z,'-b');hold on;
%% ========  Way Point 3 =========================
PX3=str2double(handles.PX3.String);
PY3=str2double(handles.PY3.String);
PZ3=str2double(handles.PZ3.String);
% inverse kinematics   
T3=[1 0 0 PX3;0 1 0 PY3;0 0 1 PZ3;0 0 0 1];
q3=r.ikine(T3, [0 0 0], 'mask',[1 1 1 0 0 0])*180/pi

% Trajectery Generarion to rotate q1 only from 1st point to 2nd point
q2_0=[q3(1) q2(2) q2(3) q2(4)];

R3=sqrt(PX3^3 +PY3^3);
int=1;qstep=q2(1):int:q2_0(1);
x=R3*cos(qstep*pi/180)
y=R3*sin(qstep*pi/180)
for i=1:length(x)
    z(i)=PZ3
end
figure(30);plot3(x,y,z,'-r');hold on;
%% ========  Way Point 4 =========================
PX4=str2double(handles.PX4.String);
PY4=str2double(handles.PY4.String);
PZ4=str2double(handles.PZ4.String);
% inverse kinematics   
T4=[1 0 0 PX4;0 1 0 PY4;0 0 1 PZ4;0 0 0 1];
q4=r.ikine(T4, [0 0 0], 'mask',[1 1 1 0 0 0])*180/pi

% Trajectery Generarion to rotate q1 only from 1st point to 2nd point
q3_0=[q4(1) q3(2) q3(3) q3(4)];

R4=sqrt(PX4^3 +PY4^3);
int=1;qstep=q3(1):int:q3_0(1);
x=R4*cos(qstep*pi/180);y=R4*sin(qstep*pi/180);
for i=1:length(x)
    z(i)=PZ4;
end
figure(30);plot3(x,y,z,'-r');hold on;
%% ========  Way Point 5 =========================
PX5=str2double(handles.PX5.String);
PY5=str2double(handles.PY5.String);
PZ5=str2double(handles.PZ5.String);
% inverse kinematics   
T5=[1 0 0 PX5;0 1 0 PY5;0 0 1 PZ5;0 0 0 1];
q5=r.ikine(T5, [0 0 0], 'mask',[1 1 1 0 0 0])*180/pi

% Trajectery Generarion to rotate q1 only from 1st point to 2nd point
q4_0=[q5(1) q4(2) q4(3) q4(4)];

R5=sqrt(PX5^3 +PY5^3);
int=1;qstep=q4(1):int:q4_0(1);
x=R5*cos(qstep*pi/180);y=R5*sin(qstep*pi/180);
for i=1:length(x)
    z(i)=PZ5;
end
figure(30);plot3(x,y,z,'-r');hold on;



function inter_Callback(hObject, eventdata, handles)
% hObject    handle to inter (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of inter as text
%        str2double(get(hObject,'String')) returns contents of inter as a double


% --- Executes during object creation, after setting all properties.
function inter_CreateFcn(hObject, eventdata, handles)
% hObject    handle to inter (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in btn_inter.
function btn_inter_Callback(hObject, eventdata, handles)
% hObject    handle to btn_inter (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
inter=str2double(handles.inter.String);
handles.inter.String = num2str((floor(inter)));



function out_Callback(hObject, eventdata, handles)
% hObject    handle to out (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of out as text
%        str2double(get(hObject,'String')) returns contents of out as a double


% --- Executes during object creation, after setting all properties.
function out_CreateFcn(hObject, eventdata, handles)
% hObject    handle to out (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on slider movement.
function slider1_Callback(hObject, eventdata, handles)
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function slider1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on button press in btn_plot.
function btn_plot_Callback(hObject, eventdata, handles)
% hObject    handle to btn_plot (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

PX1=str2double(handles.PX1.String);PX2=str2double(handles.PX2.String);
PY1=str2double(handles.PY1.String);PY2=str2double(handles.PY2.String);
PZ1=str2double(handles.PZ1.String);PZ2=str2double(handles.PZ2.String); 

%create workspace for half donut
%workshop_workspace
        u=linspace(0,pi,100);
        v=linspace(0,2*pi,100);
        [U,V]=meshgrid(u,v);
        X=(3+cos(V))*cos(U)/400*1000;
        Y=(3+cos(V))*sin(U)/400*1000;
        Z=(sin(V)/4+0.5)*1000;
        surf(X,Y,Z,'FaceAlpha',0.5,'EdgeColor','none');hold on; %creates a semitransparent surface with no edges drawn. 
        %CO(:,:,1) = zeros(25); % red
        %CO(:,:,2) = ones(25).*linspace(0.5,0.6,25); % green
        %CO(:,:,3) = ones(25).*linspace(0,1,25); % blue
        %surf(X,Y,Z,CO);

figure(2);title('Work Space')
        subplot(2,2,1);surf(X,Y,Z,'FaceAlpha',0.5,'EdgeColor','none');hold on; plot3(PX1,PY1,PZ1,'or');plot3(PX2,PY2,PZ2,'ob');
        subplot(2,2,2);plot(X,Y);title('X vs. Y');xlabel('X-Cordinate'); ylabel('Y_cordinate');hold on;plot(PX1,PY1,'or');plot(PX2,PY2,'ob');
        subplot(2,2,3); plot(X,Z);title('X vs. Z');xlabel('X-Cordinate'); ylabel('Z_cordinate');hold on;plot(PX1,PZ1,'or');plot(PX2,PZ2,'ob');
        subplot(2,2,4); plot(Y,Z);title('Y vs. Z');xlabel('Y-Cordinate'); ylabel('Z_cordinate');hold on;plot(PY1,PZ1,'or');plot(PY2,PZ2,'ob');


% --- Executes on button press in btn_radius_move.
function btn_radius_move_Callback(hObject, eventdata, handles)
% hObject    handle to btn_radius_move (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%% ======  Read & WriteDH parameters from input ========
dh_robot
load_points
   temp=0:2:50; %st=50;
%% ===============  Start Point 1 =========================
PX=str2double(handles.Pos_X.String);PY=str2double(handles.Pos_Y.String);PZ=str2double(handles.Pos_Z.String);
% inverse kinematics   
T0=[1 0 0 PX;0 1 0 PY;0 0 1 PZ;0 0 0 1];
q0=rob.ikine(T0, [0 0 0], 'mask',[1 1 1 0 0 0]);%*180/pi;
if q0(1) < 0
    q0(1)=pi+q0(1);
end
%% ===============  Way Point 1 =========================
PX1=str2double(handles.PX1.String);PY1=str2double(handles.PY1.String);PZ1=str2double(handles.PZ1.String);
lim_Y=L1+L2+L3+L4+L5+L6;
if PY1 > lim_Y 
    handles.Message.String = num2str('Y Cordinate point is out of work space , , please enter new coordinate');
else 
end

%% ============  inverse kinematics   
T1=[1 0 0 PX1;0 1 0 PY1;0 0 1 PZ1;0 0 0 1];
q1=rob.ikine(T1, [0 0 0], 'mask',[1 1 1 0 0 0]);%*180/pi;
if q1(1) < 0
    q1(1)=pi+q1(1);
    out=q1(1);
end
%% valid point check
%valid_points
%% Trajctory Generation 1
rplot=sqrt(PX^2 + PY^2);rplot=abs(rplot);
int=q1(1)/inter;th=q0(1):int:q1(1);xp=rplot*cos(th);yp=rplot*sin(th);
zp=[];
for i=1:length(xp)
    zp(i)=PZ;
end
plot3(xp,yp,zp,'-r','LineWidth',2);grid on;hold on;plot3(PX,PY,PZ,'or');hold on;plot3(PX1,PY1, PZ1,'ob'); hold on;axis([-1100 1100 -150 1000 -500 750]);

%% Move Robot by rotating q1 only on trajectory 1 and then moving on Traj 2
out=q1(1);

q0_0=[q1(1) q0(2) q0(3) q0(4) q0(5) q0(6)];q=jtraj(q0,q0_0,inter); rob.plot(q);
% q=jtraj(q0_0,q1,inter); r.plot(q);
xpp=xp(length(xp));ypp=yp(length(yp));zpp=zp(length(zp));
Tp=[1 0 0 xp;0 1 0 yp;0 0 1 zp;0 0 0 1];
T=ctraj(Tp, T1, inter);qpp=rob.ikine(T, [0 0 0], 'mask',[1 1 1 0 0 0]);%*180/pi;
rob.plot(qpp);
%% Ploting Trajectory 2
start=[xpp ypp zpp];last=[PX1 PY1 PZ1];via=[start;last];
x = mstraj(via, [10 10 10], [], start, inter, 10);  % via points, velocity, start point, timestep, acceleration
plot3(x(:,1),x(:,2),x(:,3),'-b','LineWidth',2);grid on;hold on;plot3(PX,PY,PZ,'or');hold on;plot3(PX1,PY1, PZ1,'ob');
figure (7);title('X vs. Y')
plot(xp,yp,'-r','LineWidth',2);grid on;hold on;plot(PX,PY,'or',PX1,PY1,'ob');xlabel ('X-axis');ylabel ('Y-axis');axis([-1100 1100 0 1100]);
plot(x(:,1),x(:,2),'-b','LineWidth',2);grid on; plot(PX,PY,'or',PX1,PY1,'ob');hold on;
figure (8);title('X vs. Z')
plot(xp,zp,'-r','LineWidth',2);grid on;hold on;plot(PX,PZ,'or',PX1,PZ1,'ob');xlabel ('X-axis');ylabel ('Y-axis');axis([-1100 1100 0 1100]);
plot(x(:,1),x(:,3),'-b','LineWidth',2);hold on;

 %% ========== Velocity Ploting =========
[q ,qd ,qdd] =jtraj(q0,q1,inter, [0 0 0 0 0 0], [10 10 10 10 10 10]);
figure(31);title('Joint Angles  Velocities Accelerations from Point 2 to Point 3');
        subplot(3,1,1);plot(q*180/pi);title('Joint Angle vs. time');xlabel('Time(s)');ylabel('Joint Angle (deg)');legend('q1','q2','q3','q4','q5','q6');
        subplot(3,1,2);plot(qd*180/pi);title('Joint velocities vs. time');xlabel('Time(s)');ylabel('Joint Velocities (deg/s)') 
        subplot(3,1,3);plot(qdd*180/pi);title('Joint Accelration vs. time');xlabel('Time(s)');ylabel('Joint Acceleration (deg/s^2)') 
%% Writing Joint angle
handles.Theta_1.String = num2str((floor(q1(1)*180/pi)));
handles.Theta_2.String = num2str((floor(q1(2)*180/pi)));
handles.Theta_3.String = num2str((floor(q1(3)*180/pi)));
handles.Theta_4.String = num2str((floor(q1(4)*180/pi)));
handles.Theta_5.String = num2str((floor(q1(5)*180/pi)));
handles.Theta_6.String = num2str((floor(q1(6)*180/pi)));

handles.out.String = num2str((floor(out)));


% --- Executes during object creation, after setting all properties.
function btn_robot_cnfig_CreateFcn(hObject, eventdata, handles)
% hObject    handle to btn_robot_cnfig (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called



function d1_Callback(hObject, eventdata, handles)
% hObject    handle to d1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of d1 as text
%        str2double(get(hObject,'String')) returns contents of d1 as a double


% --- Executes during object creation, after setting all properties.
function d1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to d1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function d2_Callback(hObject, eventdata, handles)
% hObject    handle to d2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of d2 as text
%        str2double(get(hObject,'String')) returns contents of d2 as a double


% --- Executes during object creation, after setting all properties.
function d2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to d2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function d3_Callback(hObject, eventdata, handles)
% hObject    handle to d3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of d3 as text
%        str2double(get(hObject,'String')) returns contents of d3 as a double


% --- Executes during object creation, after setting all properties.
function d3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to d3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function d4_Callback(hObject, eventdata, handles)
% hObject    handle to d4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of d4 as text
%        str2double(get(hObject,'String')) returns contents of d4 as a double


% --- Executes during object creation, after setting all properties.
function d4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to d4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function o1_Callback(hObject, eventdata, handles)
% hObject    handle to o1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of o1 as text
%        str2double(get(hObject,'String')) returns contents of o1 as a double


% --- Executes during object creation, after setting all properties.
function o1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to o1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function o2_Callback(hObject, eventdata, handles)
% hObject    handle to o2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of o2 as text
%        str2double(get(hObject,'String')) returns contents of o2 as a double


% --- Executes during object creation, after setting all properties.
function o2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to o2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function o3_Callback(hObject, eventdata, handles)
% hObject    handle to o3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of o3 as text
%        str2double(get(hObject,'String')) returns contents of o3 as a double


% --- Executes during object creation, after setting all properties.
function o3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to o3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function o4_Callback(hObject, eventdata, handles)
% hObject    handle to o4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of o4 as text
%        str2double(get(hObject,'String')) returns contents of o4 as a double


% --- Executes during object creation, after setting all properties.
function o4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to o4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function alpha1_Callback(hObject, eventdata, handles)
% hObject    handle to alpha1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of alpha1 as text
%        str2double(get(hObject,'String')) returns contents of alpha1 as a double


% --- Executes during object creation, after setting all properties.
function alpha1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to alpha1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function alpha2_Callback(hObject, eventdata, handles)
% hObject    handle to alpha2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of alpha2 as text
%        str2double(get(hObject,'String')) returns contents of alpha2 as a double


% --- Executes during object creation, after setting all properties.
function alpha2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to alpha2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function alpha3_Callback(hObject, eventdata, handles)
% hObject    handle to alpha3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of alpha3 as text
%        str2double(get(hObject,'String')) returns contents of alpha3 as a double


% --- Executes during object creation, after setting all properties.
function alpha3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to alpha3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function alpha4_Callback(hObject, eventdata, handles)
% hObject    handle to alpha4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of alpha4 as text
%        str2double(get(hObject,'String')) returns contents of alpha4 as a double


% --- Executes during object creation, after setting all properties.
function alpha4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to alpha4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in btn_check.
function btn_check_Callback(hObject, eventdata, handles)
% hObject    handle to btn_check (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

offset_y=str2double(handles.Y_offset.String); % read y offset from panel

vx=10000;vy=10000;vz=10000;
%  load('path.mat')            % load Only variable path_node
global X;
global Y;
x=X;y=Y;
init_goal

init_c =[init(1) init(2)+offset_y  init(3)  init(4)];
goal_c =[goal(1) goal(2)+offset_y  goal(3)  goal(4)];
axes(handles.axes1)
a = get(handles.obstacle_selection,'Value');
[Obstacle] = obstacle(a);
Obstacle_c = [Obstacle(:,1)  Obstacle(:,2)+offset_y   Obstacle(:,3)];
axis([-2200 2200 -800 4550]);
% axis tight
% axis auto
rectangle('Position',[-1000 -1000+offset_y 2000 2000],'EdgeColor','b','LineWidth',2);
viscircles(Obstacle_c(:,1:2),Obstacle_c(:,3));
circle(init_c,r_init);circle(goal_c,goal_rad);


% plot(x,y+offset_y,'*r','LineWidth',2);
dh_robot
PX = init(1);PY = init(2)+offset_y; PZ=0;
PX1 = goal(1);PY1 = goal(2)+offset_y; PZ1=0;


 

%% ===============   Transformation Matrix & Inverse Kinematics =========================
T0=[1 0 0 PX;0 1 0 PY;0 0 1 PZ;0 0 0 1];   q0=rob.ikine(T0, [0 0 0], 'mask',[1 1 1 0 0 0])*180/pi;
T1=[1 0 0 PX1;0 1 0 PY1;0 0 1 PZ1;0 0 0 1];q1=rob.ikine(T1, [0 0 0], 'mask',[1 1 1 0 0 0])*180/pi;
% joint_correction
% ====================    Robot Move & Trajectory of all Points ==================
% Robot move & Ploting Trajectory 0 to 1
start=[PX PY PZ];last=[PX1 PY1 PZ1];via=[start;last];
xj = mstraj(via, [vx vy vz], [], start, inter, 10);  % via points, velocity, start point, timestep, acceleration
% plot3(xj(:,1),xj(:,2),xj(:,3),'-c','LineWidth',1);grid on;hold on;
plot3(PX,PY,PZ,'*r');hold on;plot3(PX1,PY1, PZ1,'ob');
T=ctraj(T0, T1, inter);
qpp=rob.ikine(T, [0 0 0], 'mask',[1 1 1 0 0 0]);

if qpp == 0
    handles.Message.String = num2str('!!!!!!!! GOAL CANNOT BE ACHIEVED !!!!!!');
else
    handles.Message.String = num2str('..... GOAL ACHIEVED!!!! ...........');
    rob.plot(qpp);
end
% write_q_p(q1,PX1,PY1,PZ1)

% handles.Message.String = num2str('..... GOAL ACHIEVED!!!! ...........');

   handles.Theta_1.String = num2str((floor(q1(1))));
   handles.Theta_2.String = num2str((floor(q1(2))));
   handles.Theta_3.String = num2str((floor(q1(3))));
   handles.Theta_4.String = num2str((floor(q1(4))));
   handles.Theta_5.String = num2str((floor(q1(5))));
   handles.Theta_6.String = num2str((floor(q1(6))));
   
   handles.Pos_X.String = num2str((floor(PX1)));
   handles.Pos_Y.String = num2str((floor(PY1)));
   handles.Pos_Z.String = num2str((floor(PZ1)));
   
%    PX = PX1; PY = PY1; PZ=0;



function Link_5_Callback(hObject, eventdata, handles)
% hObject    handle to Link_5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Link_5 as text
%        str2double(get(hObject,'String')) returns contents of Link_5 as a double


% --- Executes during object creation, after setting all properties.
function Link_5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Link_5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function d5_Callback(hObject, eventdata, handles)
% hObject    handle to d5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of d5 as text
%        str2double(get(hObject,'String')) returns contents of d5 as a double


% --- Executes during object creation, after setting all properties.
function d5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to d5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function o5_Callback(hObject, eventdata, handles)
% hObject    handle to o5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of o5 as text
%        str2double(get(hObject,'String')) returns contents of o5 as a double


% --- Executes during object creation, after setting all properties.
function o5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to o5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function alpha5_Callback(hObject, eventdata, handles)
% hObject    handle to alpha5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of alpha5 as text
%        str2double(get(hObject,'String')) returns contents of alpha5 as a double


% --- Executes during object creation, after setting all properties.
function alpha5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to alpha5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Link_6_Callback(hObject, eventdata, handles)
% hObject    handle to Link_6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Link_6 as text
%        str2double(get(hObject,'String')) returns contents of Link_6 as a double


% --- Executes during object creation, after setting all properties.
function Link_6_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Link_6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function d6_Callback(hObject, eventdata, handles)
% hObject    handle to d6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of d6 as text
%        str2double(get(hObject,'String')) returns contents of d6 as a double


% --- Executes during object creation, after setting all properties.
function d6_CreateFcn(hObject, eventdata, handles)
% hObject    handle to d6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function o6_Callback(hObject, eventdata, handles)
% hObject    handle to o6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of o6 as text
%        str2double(get(hObject,'String')) returns contents of o6 as a double


% --- Executes during object creation, after setting all properties.
function o6_CreateFcn(hObject, eventdata, handles)
% hObject    handle to o6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function alpha6_Callback(hObject, eventdata, handles)
% hObject    handle to alpha6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of alpha6 as text
%        str2double(get(hObject,'String')) returns contents of alpha6 as a double


% --- Executes during object creation, after setting all properties.
function alpha6_CreateFcn(hObject, eventdata, handles)
% hObject    handle to alpha6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Theta_5_Callback(hObject, eventdata, handles)
% hObject    handle to Theta_5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Theta_5 as text
%        str2double(get(hObject,'String')) returns contents of Theta_5 as a double


% --- Executes during object creation, after setting all properties.
function Theta_5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Theta_5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Theta_6_Callback(hObject, eventdata, handles)
% hObject    handle to Theta_6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Theta_6 as text
%        str2double(get(hObject,'String')) returns contents of Theta_6 as a double


% --- Executes during object creation, after setting all properties.
function Theta_6_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Theta_6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in btn_robot_cnfig.
function btn_plot_traj_Callback(hObject, eventdata, handles)
% hObject    handle to btn_robot_cnfig (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
dh_robot
load_points
inter=str2double(handles.inter.String);

%% ===============   Transformation Matrix & Inverse Kinematics =========================
T0=[1 0 0 PX;0 1 0 PY;0 0 1 PZ;0 0 0 1];   q0=rob.ikine(T0, [0 0 0], 'mask',[1 1 1 0 0 0]);%*180/pi;
T1=[1 0 0 PX1;0 1 0 PY1;0 0 1 PZ1;0 0 0 1];q1=rob.ikine(T1, [0 0 0], 'mask',[1 1 1 0 0 0]);%*180/pi;
% T2=[1 0 0 PX2;0 1 0 PY2;0 0 1 PZ2;0 0 0 1];q2=rob.ikine(T2, [0 0 0], 'mask',[1 1 1 0 0 0]);%*180/pi;
% T3=[1 0 0 PX3;0 1 0 PY3;0 0 1 PZ3;0 0 0 1];q3=rob.ikine(T3, [0 0 0], 'mask',[1 1 1 0 0 0]);%*180/pi;
% T4=[1 0 0 PX4;0 1 0 PY4;0 0 1 PZ4;0 0 0 1];q4=rob.ikine(T4, [0 0 0], 'mask',[1 1 1 0 0 0]);%*180/pi;
% T5=[1 0 0 PX5;0 1 0 PY5;0 0 1 PZ5;0 0 0 1];q5=rob.ikine(T5, [0 0 0], 'mask',[1 1 1 0 0 0]);%*180/pi;

% joint_correction
%% ====================    Robot Move & Trajectory of all Points ==================
% Robot move & Ploting Trajectory 0 to 1
start=[PX PY PZ];last=[PX1 PY1 PZ1];via=[start;last];
x = mstraj(via, [10 10 10], [], start, inter, 10);  % via points, velocity, start point, timestep, acceleration
plot3(x(:,1),x(:,2),x(:,3),'-g','LineWidth',3);grid on;hold on;plot3(PX,PY,PZ,'or');hold on;plot3(PX1,PY1, PZ1,'ob');
T=ctraj(T0, T1, inter);qpp=rob.ikine(T, [0 0 0], 'mask',[1 1 1 0 0 0]);rob.plot(qpp);
write_q_p(q1,PX1,PY1,PZ1)


% --- Executes on button press in btn_robot_cnfig.
function pushbutton40_Callback(hObject, eventdata, handles)
% hObject    handle to btn_robot_cnfig (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on selection change in obstacle_selection.
function obstacle_selection_Callback(hObject, eventdata, handles)
% hObject    handle to obstacle_selection (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns obstacle_selection contents as cell array
%        contents{get(hObject,'Value')} returns selected item from obstacle_selection
clf(figure);
a = get(handles.obstacle_selection,'Value');

    [Obstacle] = obstacle(a);
    init_goal
%     [init , goal ] = init_goal()
     plot_obstacle( init,goal,r_init, goal_rad,Obstacle )
%    plot_workspace( work_space,Obstacle,init,r_init,goal,goal_rad )


% --- Executes during object creation, after setting all properties.
function obstacle_selection_CreateFcn(hObject, eventdata, handles)
% hObject    handle to obstacle_selection (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function x_initial_Callback(hObject, eventdata, handles)
% hObject    handle to x_initial (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of x_initial as text
%        str2double(get(hObject,'String')) returns contents of x_initial as a double


% --- Executes during object creation, after setting all properties.
function x_initial_CreateFcn(hObject, eventdata, handles)
% hObject    handle to x_initial (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function y_initial_Callback(hObject, eventdata, handles)
% hObject    handle to y_initial (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of y_initial as text
%        str2double(get(hObject,'String')) returns contents of y_initial as a double


% --- Executes during object creation, after setting all properties.
function y_initial_CreateFcn(hObject, eventdata, handles)
% hObject    handle to y_initial (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function z_initial_Callback(hObject, eventdata, handles)
% hObject    handle to z_initial (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of z_initial as text
%        str2double(get(hObject,'String')) returns contents of z_initial as a double


% --- Executes during object creation, after setting all properties.
function z_initial_CreateFcn(hObject, eventdata, handles)
% hObject    handle to z_initial (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function x_goal_Callback(hObject, eventdata, handles)
% hObject    handle to x_goal (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of x_goal as text
%        str2double(get(hObject,'String')) returns contents of x_goal as a double


% --- Executes during object creation, after setting all properties.
function x_goal_CreateFcn(hObject, eventdata, handles)
% hObject    handle to x_goal (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function y_goal_Callback(hObject, eventdata, handles)
% hObject    handle to y_goal (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of y_goal as text
%        str2double(get(hObject,'String')) returns contents of y_goal as a double


% --- Executes during object creation, after setting all properties.
function y_goal_CreateFcn(hObject, eventdata, handles)
% hObject    handle to y_goal (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function z_goal_Callback(hObject, eventdata, handles)
% hObject    handle to z_goal (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of z_goal as text
%        str2double(get(hObject,'String')) returns contents of z_goal as a double


% --- Executes during object creation, after setting all properties.
function z_goal_CreateFcn(hObject, eventdata, handles)
% hObject    handle to z_goal (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in update_cord.
function update_cord_Callback(hObject, eventdata, handles)
% hObject    handle to update_cord (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

init(1)=str2double(handles.x_initial.String);
init(2)=str2double(handles.y_initial.String);
init(3)=str2double(handles.z_initial.String);
goal(1)=str2double(handles.x_goal.String);
goal(2)=str2double(handles.y_goal.String);
goal(3)=str2double(handles.z_goal.String);
offset_y=str2double(handles.Y_offset.String);

handles.x_initial.String = num2str((floor(init(1))));
handles.y_initial.String = num2str((floor(init(2))));
handles.z_initial.String = num2str((floor(init(3))));
handles.x_goal.String    = num2str((floor(goal(1))));
handles.y_goal.String    = num2str((floor(goal(2))));
handles.z_goal.String    = num2str((floor(goal(3))));
handles.Y_offset.String  = num2str((floor(offset_y)));

if offset_y < 1200
    handles.Message.String = num2str('Error:....Enter Robot offset > 1200  ....');
end


function r_init_Callback(hObject, eventdata, handles)
% hObject    handle to r_init (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of r_init as text
%        str2double(get(hObject,'String')) returns contents of r_init as a double


% --- Executes during object creation, after setting all properties.
function r_init_CreateFcn(hObject, eventdata, handles)
% hObject    handle to r_init (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function goal_rad_Callback(hObject, eventdata, handles)
% hObject    handle to goal_rad (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of goal_rad as text
%        str2double(get(hObject,'String')) returns contents of goal_rad as a double


% --- Executes during object creation, after setting all properties.
function goal_rad_CreateFcn(hObject, eventdata, handles)
% hObject    handle to goal_rad (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Y_offset_Callback(hObject, eventdata, handles)
% hObject    handle to Y_offset (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Y_offset as text
%        str2double(get(hObject,'String')) returns contents of Y_offset as a double


% --- Executes during object creation, after setting all properties.
function Y_offset_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Y_offset (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in dh_par_sel.
function dh_par_sel_Callback(hObject, eventdata, handles)
% hObject    handle to dh_par_sel (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns dh_par_sel contents as cell array
%        contents{get(hObject,'Value')} returns selected item from dh_par_sel

b = get(handles.dh_par_sel,'Value')

robot_config(b)
read_para_dh
dh_robot
workspace
init_goal;                   % load initial & goal coordinates & radius
PX=P(1);PY=P(2);PZ=P(3);

T=[1 0 0 PX;
    0 1 0 PY;
    0 0 1 PZ;
    0 0 0 1];

%returns joint angles for given home pose 
q=rob.ikine(T, [0 0 0], 'mask',[1 1 1 0 0 0])*180/pi;
axes(handles.axes1)
rob.plot(q*pi/180);hold on;
write_q_p


        
        

% --- Executes during object creation, after setting all properties.
function dh_par_sel_CreateFcn(hObject, eventdata, handles)
% hObject    handle to dh_par_sel (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function curve_para_Callback(hObject, eventdata, handles)
% hObject    handle to curve_para (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of curve_para as text
%        str2double(get(hObject,'String')) returns contents of curve_para as a double
curve_para=str2double(handles.curve_para.String);
handles.curve_para.String    = num2str((floor(curve_para)));


% --- Executes during object creation, after setting all properties.
function curve_para_CreateFcn(hObject, eventdata, handles)
% hObject    handle to curve_para (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



% --- Executes on button press in button_path_smooth.
function button_path_smooth_Callback(hObject, eventdata, handles)
% hObject    handle to button_path_smooth (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

offset_y=str2double(handles.Y_offset.String); % read y offset from panel
curve_para=str2double(handles.curve_para.String); % read y offset from panel
smooth_per_edit   = str2double(handles.smooth_per_edit.String);
vx=10000;vy=10000;vz=10000;
%  load('path.mat')            % load Only variable path_node
global X;
global Y;
x=X;y=Y;
init_goal

init_c =[init(1) init(2)+offset_y  init(3)  init(4)];
goal_c =[goal(1) goal(2)+offset_y  goal(3)  goal(4)];
a = get(handles.obstacle_selection,'Value');
[Obstacle] = obstacle(a);
Obstacle_c = [Obstacle(:,1)  Obstacle(:,2)+offset_y   Obstacle(:,3)];

% figure (33);
axes(handles.axes6)

tic
% [fitresult, ft_path] = createFit(x, y, curve_para);
iter_smooth       = str2double(handles.iter_smooth.String);
[ n_pts, pts, xx , yy ] = path_smooth( x , y,iter_smooth,Obstacle,clearance,base_radius,smooth_per_edit );
toc
h1 = msgbox({'Total smoothing in seconds.........' '' num2str(toc)} ,'Info');
handles.path_time.String = num2str((floor(toc)));
uiwait(h1,10)

[ pathcost ] = path_cost( xx,yy );
handles.del_pts.String = num2str((floor(pts)));
handles.n_pts.String = num2str((floor(n_pts)));
X =xx;Y=yy;
handles.pathcost.String = num2str((floor(pathcost)));
plot( xx, yy,'-g','LineWidth',3,'MarkerFaceColor','k', 'MarkerSize',20 ); %hold on;
plot(xx,yy,'m+');
% Plot fit with data.

% plot( fitresult);hold on;
% figure (12);



% plot( x , y ,'-b' );% hold off;
% plot( x, ft_path,'-g' );
% save('path_fit.mat','x','ft_path')

% --- Executes during object creation, after setting all properties.
function button_path_smooth_CreateFcn(hObject, eventdata, handles)
% hObject    handle to elapsed_time (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function NN_Callback(hObject, eventdata, handles)
% hObject    handle to NN (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of NN as text
%        str2double(get(hObject,'String')) returns contents of NN as a double

NN=str2double(handles.NN.String)
handles.NN.String    = num2str((floor(NN)));


% --- Executes during object creation, after setting all properties.
function NN_CreateFcn(hObject, eventdata, handles)
% hObject    handle to NN (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end




% --- Executes on button press in pushbutton_iter.
function pushbutton_iter_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_iter (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


iter=str2double(handles.curve_para.String);
handles.iter.String    = num2str((floor(iter)));



function elapsed_time_Callback(hObject, eventdata, handles)
% hObject    handle to elapsed_time (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of elapsed_time as text
%        str2double(get(hObject,'String')) returns contents of elapsed_time as a double


% --- Executes during object creation, after setting all properties.
function elapsed_time_CreateFcn(hObject, eventdata, handles)
% hObject    handle to elapsed_time (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function path_time_Callback(hObject, eventdata, handles)
% hObject    handle to path_time (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of path_time as text
%        str2double(get(hObject,'String')) returns contents of path_time as a double


% --- Executes during object creation, after setting all properties.
function path_time_CreateFcn(hObject, eventdata, handles)
% hObject    handle to path_time (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function random_Callback(hObject, eventdata, handles)
% hObject    handle to random (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of random as text
%        str2double(get(hObject,'String')) returns contents of random as a double


% --- Executes during object creation, after setting all properties.
function random_CreateFcn(hObject, eventdata, handles)
% hObject    handle to random (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function iter_NN_Callback(hObject, eventdata, handles)
% hObject    handle to iter_NN (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of iter_NN as text
%        str2double(get(hObject,'String')) returns contents of iter_NN as a double


% --- Executes during object creation, after setting all properties.
function iter_NN_CreateFcn(hObject, eventdata, handles)
% hObject    handle to iter_NN (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function min_cost_Callback(hObject, eventdata, handles)
% hObject    handle to min_cost (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of min_cost as text
%        str2double(get(hObject,'String')) returns contents of min_cost as a double


% --- Executes during object creation, after setting all properties.
function min_cost_CreateFcn(hObject, eventdata, handles)
% hObject    handle to min_cost (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in clearfig.
function clearfig_Callback(hObject, eventdata, handles)
% hObject    handle to clearfig (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
axes(handles.axes6);
title('Refresh');
axis([-1050 1050 -1050 1050]);hold on;
rectangle('Position',[-1000 -1000 2000 2000],'EdgeColor','m','LineWidth',2);

% clf('reset')
hold off axes(handles.axes6);
% clf (axes(handles.axes6));
clf (figure(12));
% close figure(12);



function near_radius_Callback(hObject, eventdata, handles)
% hObject    handle to near_radius (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of near_radius as text
%        str2double(get(hObject,'String')) returns contents of near_radius as a double


% --- Executes during object creation, after setting all properties.
function near_radius_CreateFcn(hObject, eventdata, handles)
% hObject    handle to near_radius (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function random_band_Callback(hObject, eventdata, handles)
% hObject    handle to random_band (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of random_band as text
%        str2double(get(hObject,'String')) returns contents of random_band as a double


% --- Executes during object creation, after setting all properties.
function random_band_CreateFcn(hObject, eventdata, handles)
% hObject    handle to random_band (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function rand_fact_Callback(hObject, eventdata, handles)
% hObject    handle to rand_fact (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of rand_fact as text
%        str2double(get(hObject,'String')) returns contents of rand_fact as a double


% --- Executes during object creation, after setting all properties.
function rand_fact_CreateFcn(hObject, eventdata, handles)
% hObject    handle to rand_fact (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function rand_wt_Callback(hObject, eventdata, handles)
% hObject    handle to rand_wt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of rand_wt as text
%        str2double(get(hObject,'String')) returns contents of rand_wt as a double


% --- Executes during object creation, after setting all properties.
function rand_wt_CreateFcn(hObject, eventdata, handles)
% hObject    handle to rand_wt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function delta_heru_Callback(hObject, eventdata, handles)
% hObject    handle to delta_heru (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of delta_heru as text
%        str2double(get(hObject,'String')) returns contents of delta_heru as a double


% --- Executes during object creation, after setting all properties.
function delta_heru_CreateFcn(hObject, eventdata, handles)
% hObject    handle to delta_heru (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function delta_fact_Callback(hObject, eventdata, handles)
% hObject    handle to delta_fact (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of delta_fact as text
%        str2double(get(hObject,'String')) returns contents of delta_fact as a double


% --- Executes during object creation, after setting all properties.
function delta_fact_CreateFcn(hObject, eventdata, handles)
% hObject    handle to delta_fact (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function n_rad_limit_Callback(hObject, eventdata, handles)
% hObject    handle to n_rad_limit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of n_rad_limit as text
%        str2double(get(hObject,'String')) returns contents of n_rad_limit as a double


% --- Executes during object creation, after setting all properties.
function n_rad_limit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to n_rad_limit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Curr_weight_Callback(hObject, eventdata, handles)
% hObject    handle to Curr_weight (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Curr_weight as text
%        str2double(get(hObject,'String')) returns contents of Curr_weight as a double


% --- Executes during object creation, after setting all properties.
function Curr_weight_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Curr_weight (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function cur_n_rad_Callback(hObject, eventdata, handles)
% hObject    handle to cur_n_rad (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of cur_n_rad as text
%        str2double(get(hObject,'String')) returns contents of cur_n_rad as a double


% --- Executes during object creation, after setting all properties.
function cur_n_rad_CreateFcn(hObject, eventdata, handles)
% hObject    handle to cur_n_rad (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function cur_n_band_Callback(hObject, eventdata, handles)
% hObject    handle to cur_n_band (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of cur_n_band as text
%        str2double(get(hObject,'String')) returns contents of cur_n_band as a double


% --- Executes during object creation, after setting all properties.
function cur_n_band_CreateFcn(hObject, eventdata, handles)
% hObject    handle to cur_n_band (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function temperature_Callback(hObject, eventdata, handles)
% hObject    handle to temperature (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of temperature as text
%        str2double(get(hObject,'String')) returns contents of temperature as a double


% --- Executes during object creation, after setting all properties.
function temperature_CreateFcn(hObject, eventdata, handles)
% hObject    handle to temperature (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function curr_delta_Callback(hObject, eventdata, handles)
% hObject    handle to curr_delta (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of curr_delta as text
%        str2double(get(hObject,'String')) returns contents of curr_delta as a double


% --- Executes during object creation, after setting all properties.
function curr_delta_CreateFcn(hObject, eventdata, handles)
% hObject    handle to curr_delta (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function del_pts_Callback(hObject, eventdata, handles)
% hObject    handle to del_pts (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of del_pts as text
%        str2double(get(hObject,'String')) returns contents of del_pts as a double


% --- Executes during object creation, after setting all properties.
function del_pts_CreateFcn(hObject, eventdata, handles)
% hObject    handle to del_pts (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function iter_smooth_Callback(hObject, eventdata, handles)
% hObject    handle to iter_smooth (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of iter_smooth as text
%        str2double(get(hObject,'String')) returns contents of iter_smooth as a double


% --- Executes during object creation, after setting all properties.
function iter_smooth_CreateFcn(hObject, eventdata, handles)
% hObject    handle to iter_smooth (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function n_pts_Callback(hObject, eventdata, handles)
% hObject    handle to n_pts (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of n_pts as text
%        str2double(get(hObject,'String')) returns contents of n_pts as a double


% --- Executes during object creation, after setting all properties.
function n_pts_CreateFcn(hObject, eventdata, handles)
% hObject    handle to n_pts (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function bf_pts_Callback(hObject, eventdata, handles)
% hObject    handle to bf_pts (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of bf_pts as text
%        str2double(get(hObject,'String')) returns contents of bf_pts as a double


% --- Executes during object creation, after setting all properties.
function bf_pts_CreateFcn(hObject, eventdata, handles)
% hObject    handle to bf_pts (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function pathcost_Callback(hObject, eventdata, handles)
% hObject    handle to pathcost (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of pathcost as text
%        str2double(get(hObject,'String')) returns contents of pathcost as a double


% --- Executes during object creation, after setting all properties.
function pathcost_CreateFcn(hObject, eventdata, handles)
% hObject    handle to pathcost (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in break_loop.
function break_loop_Callback(hObject, eventdata, handles)
% hObject    handle to break_loop (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
clearvars

return


% --- Executes during object creation, after setting all properties.
function btn_load_traj_CreateFcn(hObject, eventdata, handles)
% hObject    handle to btn_load_traj (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called



function iter_Callback(hObject, eventdata, handles)
% hObject    handle to iter (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of iter as text
%        str2double(get(hObject,'String')) returns contents of iter as a double


% --- Executes during object creation, after setting all properties.
function iter_CreateFcn(hObject, eventdata, handles)
% hObject    handle to iter (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in dynamic_button.
function dynamic_button_Callback(hObject, eventdata, handles)
% hObject    handle to dynamic_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
vx=0;vy=-4.0;vz=0.0;
handles.vx.String = num2str(floor(vx),3);
handles.vy.String = num2str(floor(vy),3);
handles.vz.String = num2str(floor(vz),3);
% Hint: get(hObject,'Value') returns toggle state of dynamic_button



function vx_Callback(hObject, eventdata, handles)
% hObject    handle to vx (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of vx as text
%        str2double(get(hObject,'String')) returns contents of vx as a double


% --- Executes during object creation, after setting all properties.
function vx_CreateFcn(hObject, eventdata, handles)
% hObject    handle to vx (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function vy_Callback(hObject, eventdata, handles)
% hObject    handle to vy (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of vy as text
%        str2double(get(hObject,'String')) returns contents of vy as a double


% --- Executes during object creation, after setting all properties.
function vy_CreateFcn(hObject, eventdata, handles)
% hObject    handle to vy (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function vz_Callback(hObject, eventdata, handles)
% hObject    handle to vz (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of vz as text
%        str2double(get(hObject,'String')) returns contents of vz as a double


% --- Executes during object creation, after setting all properties.
function vz_CreateFcn(hObject, eventdata, handles)
% hObject    handle to vz (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in Dynamic_Obstacle_button.
function Dynamic_Obstacle_button_Callback(hObject, eventdata, handles)
% hObject    handle to Dynamic_Obstacle_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of Dynamic_Obstacle_button

vx_o=-0.01;vy_o=0.0;vz_o=0.0;
handles.vx_o.String = num2str(floor(vx_o),3);
handles.vy_o.String = num2str(floor(vy_o),3);
handles.vz_o.String = num2str(floor(vz_o),3);



function vx_o_Callback(hObject, eventdata, handles)
% hObject    handle to vx_o (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of vx_o as text
%        str2double(get(hObject,'String')) returns contents of vx_o as a double


% --- Executes during object creation, after setting all properties.
function vx_o_CreateFcn(hObject, eventdata, handles)
% hObject    handle to vx_o (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function vy_o_Callback(hObject, eventdata, handles)
% hObject    handle to vy_o (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of vy_o as text
%        str2double(get(hObject,'String')) returns contents of vy_o as a double


% --- Executes during object creation, after setting all properties.
function vy_o_CreateFcn(hObject, eventdata, handles)
% hObject    handle to vy_o (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function vz_o_Callback(hObject, eventdata, handles)
% hObject    handle to vz_o (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of vz_o as text
%        str2double(get(hObject,'String')) returns contents of vz_o as a double


% --- Executes during object creation, after setting all properties.
function vz_o_CreateFcn(hObject, eventdata, handles)
% hObject    handle to vz_o (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function smooth_per_edit_Callback(hObject, eventdata, handles)
% hObject    handle to smooth_per_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of smooth_per_edit as text
%        str2double(get(hObject,'String')) returns contents of smooth_per_edit as a double


% --- Executes during object creation, after setting all properties.
function smooth_per_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to smooth_per_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in Path_Search_Method.
function Path_Search_Method_Callback(hObject, eventdata, handles)
% hObject    handle to Path_Search_Method (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns Path_Search_Method contents as cell array
%        contents{get(hObject,'Value')} returns selected item from Path_Search_Method

c = get(handles.Path_Search_Method,'Value');

switch c
    case 1
        near_rad=10.0;
        band=20;  % cirular radius of near radius & randomness
        fact=10.0;
        wt=10.0;   % wt used for near radius & random # gen if no near pt found
        delta=0.00;
        fact_d=12;% delta added  in heuristic func to comes out of local minima & factor if stucked
        near_rad_limit=260;
        
        handles.near_radius.String      = num2str((floor(near_rad)));
        handles.random_band.String      = num2str((floor(band)));
        handles.rand_fact.String        = num2str((floor(fact)));
        handles.rand_wt.String          = num2str((floor(wt)));
        handles.delta_heru.String       = num2str((floor(delta)));
        handles.delta_fact.String       = num2str((floor(fact_d)));
        handles.n_rad_limit.String      = num2str((floor(near_rad_limit)));
    case 2
        near_rad=12.0;
        band=10.0;  % cirular radius of near radius & randomness
        wt= 7.0;   % wt used for near radius & random # gen if no near pt found
        fact=10.0;
        delta=0.00;
        fact_d=15;% delta added  in heuristic func to comes out of local minima & factor if stucked
        near_rad_limit=260;
        
        handles.near_radius.String      = num2str((floor(near_rad)));
        handles.random_band.String      = num2str((floor(band)));
        handles.rand_fact.String        = num2str((floor(fact)));
        handles.rand_wt.String          = num2str((floor(wt)));
        handles.delta_heru.String       = num2str((floor(delta)));
        handles.delta_fact.String       = num2str((floor(fact_d)));
        handles.n_rad_limit.String      = num2str((floor(near_rad_limit)));
    case 3   
        

end
         



% --- Executes during object creation, after setting all properties.
function Path_Search_Method_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Path_Search_Method (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes during object creation, after setting all properties.
function uipanel11_CreateFcn(hObject, eventdata, handles)
% hObject    handle to uipanel11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
