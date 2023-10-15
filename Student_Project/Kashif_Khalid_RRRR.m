function varargout = Kashif_Khalid_RRRR(varargin)
% KASHIF_KHALID_RRRR MATLAB code for Kashif_Khalid_RRRR.fig
%      KASHIF_KHALID_RRRR, by itself, creates a new KASHIF_KHALID_RRRR or raises the existing
%      singleton*.
%
%      H = KASHIF_KHALID_RRRR returns the handle to a new KASHIF_KHALID_RRRR or the handle to
%      the existing singleton*.
%
%      KASHIF_KHALID_RRRR('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in KASHIF_KHALID_RRRR.M with the given input arguments.
%
%      KASHIF_KHALID_RRRR('Property','Value',...) creates a new KASHIF_KHALID_RRRR or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before Kashif_Khalid_RRRR_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to Kashif_Khalid_RRRR_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help Kashif_Khalid_RRRR

% Last Modified by GUIDE v2.5 28-Mar-2020 17:11:42

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @Kashif_Khalid_RRRR_OpeningFcn, ...
                   'gui_OutputFcn',  @Kashif_Khalid_RRRR_OutputFcn, ...
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


% --- Executes just before Kashif_Khalid_RRRR is made visible.
function Kashif_Khalid_RRRR_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to Kashif_Khalid_RRRR (see VARARGIN)

% Choose default command line output for Kashif_Khalid_RRRR
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes Kashif_Khalid_RRRR wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = Kashif_Khalid_RRRR_OutputFcn(hObject, eventdata, handles) 
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
%    workshop_workspace
% PX=P(1);PY=P(2);PZ=0;
   donut_workspace
   PX=700;PY=0;PZ=500;
   
   T=[1 0 0 PX;
      0 1 0 PY;
      0 0 1 PZ;
      0 0 0 1]
%returns joint angles for given home pose
q=rob.ikine(T, [0 0 0], 'mask',[1 1 1 0 0 0])*180/pi;
rob.plot(q*pi/180);hold on;
% rob.plot(q);hold on;

   handles.Theta_1.String = num2str((floor(q(1))));
   handles.Theta_2.String = num2str((floor(q(2))));
   handles.Theta_3.String = num2str((floor(q(3))));
   handles.Theta_4.String = num2str((floor(q(4))));
   handles.Theta_5.String = num2str((floor(q(5))));
   handles.Theta_6.String = num2str((floor(q(6))));
   
   handles.Pos_X.String = num2str((floor(PX)));
   handles.Pos_Y.String = num2str((floor(PY)));
   handles.Pos_Z.String = num2str((floor(PZ)));
   
    LPx=[P1(1) P2(1) P3(1) P4(1) P1(1)];
    LPy=[P1(2) P2(2) P3(2) P4(2) P1(2)];
    %LPz=[0 0 0 0]
    plot( LPx,LPy,'-r');
 
% --- Executes on button press in btn_start.
function btn_start_Callback(hObject, eventdata, handles)
% hObject    handle to btn_start (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
inter=10;
% L1=100;  L2=200;  L3=300;  L4=200;  L5=150;  L6=100;  d1=0;d2=0;d3=0;d4=0;d5=0;d6=0;
% alpha1=0;alpha2=90;alpha3=0;alpha4=0;alpha5=0;alpha6=0;o1=0;o2=0;o3=0;o4=0;o5=0;o6=0;

L1=0;  L2=450;  L3=400;  L4=350;  L5=0;  L6=0;  d1=5;d2=0;d3=0;d4=0;d5=0;d6=0;
alpha1=-90;alpha2=0;alpha3=0;alpha4=0;alpha5=0;alpha6=0;o1=0;o2=0;o3=0;o4=0;o5=0;o6=0;

handles.inter.String = num2str((floor(inter)));
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


% --- Executes on button press in btn_workspace.
function btn_workspace_Callback(hObject, eventdata, handles)
% hObject    handle to btn_workspace (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
figure (11);
% workshop_workspace
% workspace_plots
% plot_base
donut_workspace



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


% --- Executes on button press in btn_robot_move.
function btn_robot_move_Callback(hObject, eventdata, handles)
% hObject    handle to btn_robot_move (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
 %% ======  Read & Write DH parameters from input ========
dh_robot
load_points
% check_points
inter=str2double(handles.inter.String);

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
start=[PX4 PY4 PZ4];last=[PX5 PY5 PZ5];via=[start;last];
x = mstraj(via, [10 10 10], [], start, inter, 10);  % via points, velocity, start point, timestep, acceleration
plot3(x(:,1),x(:,2),x(:,3),'-c','LineWidth',3);grid on;hold on;plot3(PX4,PY4,PZ4,'or');hold on;plot3(PX5,PY5, PZ5,'ob');
T=ctraj(T4, T5, inter);qpp=rob.ikine(T, [0 0 0], 'mask',[1 1 1 0 0 0]);rob.plot(qpp);
% -----------------Robot move & Ploting Trajectory 1 to 5
start=[PX5 PY5 PZ5];last=[PX1 PY1 PZ1];via=[start;last];
x = mstraj(via, [10 10 10], [], start, inter, 10);  % via points, velocity, start point, timestep, acceleration
plot3(x(:,1),x(:,2),x(:,3),'-m','LineWidth',3);grid on;hold on;plot3(PX5,PY5,PZ5,'or');hold on;plot3(PX1,PY1, PZ1,'ob');
T=ctraj(T5, T1, inter);qpp=rob.ikine(T, [0 0 0], 'mask',[1 1 1 0 0 0]);rob.plot(qpp);
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


% --- Executes on button press in btn_Points.
function btn_Points_Callback(hObject, eventdata, handles)
% hObject    handle to btn_Points (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
workshop_workspace
% PX1=P1(1);PY1=P1(2);PZ1=50;
PX1=500;PY1=500;PZ1=0;% by passed from workspace
handles.PX1.String = num2str((floor(PX1)));
handles.PY1.String = num2str((floor(PY1)));
handles.PZ1.String = num2str((floor(PZ1)));

% PX2=P2(1);PY2=P2(2);PZ2=-50;
PX2=-500;PY2=500;PZ2=0;% by passed from workspace
handles.PX2.String = num2str((floor(PX2)));
handles.PY2.String = num2str((floor(PY2)));
handles.PZ2.String = num2str((floor(PZ2)));

% PX3=P3(1);PY3=P3(2);PZ3=50;
PX3=0;PY3=500;PZ3=100;% by passed from workspace
handles.PX3.String = num2str((floor(PX3)));
handles.PY3.String = num2str((floor(PY3)));
handles.PZ3.String = num2str((floor(PZ3)));

% PX4=P4(1);PY4=P4(2);PZ4=-50;
PX4=-700;PY4=-700;PZ4=-100;% by passed from workspace
handles.PX4.String = num2str((floor(PX4)));
handles.PY4.String = num2str((floor(PY4)));
handles.PZ4.String = num2str((floor(PZ4)));

% PX5=P(1);PY5=P(2);PZ5=20;
PX5=700;PY5=700;PZ5=100;% by passed from workspace
handles.PX5.String = num2str((floor(PX5)));
handles.PY5.String = num2str((floor(PY5)));
handles.PZ5.String = num2str((floor(PZ5)));


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
function btn_robot_move_CreateFcn(hObject, eventdata, handles)
% hObject    handle to btn_robot_move (see GCBO)
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


% --- Executes on button press in btn_base_optimize.
function btn_base_optimize_Callback(hObject, eventdata, handles)
% hObject    handle to btn_base_optimize (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

workshop_workspace
figure (12);
base_optimize



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


% --- Executes on button press in btn_plot_traj.
function btn_plot_traj_Callback(hObject, eventdata, handles)
% hObject    handle to btn_plot_traj (see GCBO)
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


% --- Executes on button press in btn_plot_traj.
function pushbutton40_Callback(hObject, eventdata, handles)
% hObject    handle to btn_plot_traj (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
