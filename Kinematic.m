function varargout = GUI_OptimalDesign(varargin)
% GUI_OPTIMALDESIGN MATLAB code for GUI_OptimalDesign.fig
%      GUI_OPTIMALDESIGN, by itself, creates a new GUI_OPTIMALDESIGN or raises the existing
%      singleton*.
%
%      H = GUI_OPTIMALDESIGN returns the handle to a new GUI_OPTIMALDESIGN or the handle to
%      the existing singleton*.
%
%      GUI_OPTIMALDESIGN('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in GUI_OPTIMALDESIGN.M with the given input arguments.
%
%      GUI_OPTIMALDESIGN('Property','Value',...) creates a new GUI_OPTIMALDESIGN or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before GUI_OptimalDesign_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to GUI_OptimalDesign_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help GUI_OptimalDesign

% Last Modified by GUIDE v2.5 27-Dec-2022 07:09:55

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @GUI_OptimalDesign_OpeningFcn, ...
                   'gui_OutputFcn',  @GUI_OptimalDesign_OutputFcn, ...
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


% --- Executes just before GUI_OptimalDesign is made visible.
function GUI_OptimalDesign_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to GUI_OptimalDesign (see VARARGIN)

% Choose default command line output for GUI_OptimalDesign
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes GUI_OptimalDesign wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = GUI_OptimalDesign_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on slider movement.
function theta1_Callback(hObject, eventdata, handles)
% hObject    handle to theta1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
Value_1 = get(hObject, 'Value');
set(handles.theta1_,'String', num2str(Value_1));


% --- Executes during object creation, after setting all properties.
function theta1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to theta1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function theta2_Callback(hObject, eventdata, handles)
% hObject    handle to theta2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
Value_2 = get(hObject, 'Value');
set(handles.theta2_,'String', num2str(Value_2));

% --- Executes during object creation, after setting all properties.
function theta2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to theta2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end



function x_Callback(hObject, eventdata, handles)
% hObject    handle to x (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of x as text
%        str2double(get(hObject,'String')) returns contents of x as a double


% --- Executes during object creation, after setting all properties.
function x_CreateFcn(hObject, eventdata, handles)
% hObject    handle to x (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on slider movement.
function theta3_Callback(hObject, eventdata, handles)
% hObject    handle to theta3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
Value_3 = get(hObject, 'Value');
set(handles.theta3_,'String', num2str(Value_3));


% --- Executes during object creation, after setting all properties.
function theta3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to theta3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function theta4_Callback(hObject, eventdata, handles)
% hObject    handle to theta4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
Value_4 = get(hObject, 'Value');
set(handles.theta4_,'String', num2str(Value_4));


% --- Executes during object creation, after setting all properties.
function theta4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to theta4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function theta5_Callback(hObject, eventdata, handles)
% hObject    handle to theta5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
Value_5 = get(hObject, 'Value');
set(handles.theta5_,'String', num2str(Value_5));


% --- Executes during object creation, after setting all properties.
function theta5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to theta5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function theta6_Callback(hObject, eventdata, handles)
% hObject    handle to theta6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function theta6_CreateFcn(hObject, eventdata, handles)
% hObject    handle to theta6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
axis on
t1 = str2double(handles.theta1_.String)*pi/180;
t2 = str2double(handles.theta2_.String)*pi/180;
t3 = str2double(handles.theta3_.String)*pi/180;
t4 = str2double(handles.theta4_.String)*pi/180;
t5 = str2double(handles.theta5_.String)*pi/180;


set(handles.theta1,'value',str2double(handles.theta1_.String));
set(handles.theta2,'value',str2double(handles.theta2_.String));
set(handles.theta3,'value',str2double(handles.theta3_.String));
set(handles.theta4,'value',str2double(handles.theta4_.String));
set(handles.theta5,'value',str2double(handles.theta5_.String));
l1 = 251;
l2 = 225;
l3 = 150;
l5 = 60;
L(1) = Link([0  l1    0  pi/2],'standard');
L(2) = Link([0   0    l2    0],'standard');
L(3) = Link([0   0    l3    0],'standard');
L(4) = Link([0   0     0 pi/2],'standard');
L(5) = Link([0  l5     0    0],'standard');


Robot = SerialLink(L);
Robot.name = 'Robot';

Robot.plot([t1 t2 t3 t4 t5]);
axis([-700 700 -700 700 -700 700])

M = Robot.fkine([t1 t2 t3 t4 t5])
T = M(1);
trsl = transl(T);
trrt = tr2rt(T);
xx = trsl(1);
yy = trsl(2);
zz = trsl(3);

handles.x.String = num2str(round(xx,5));
handles.y.String = num2str(round(yy,5));
handles.z.String = num2str(round(zz,5));
rx = acos(trrt(1,1)/sqrt(trrt(1,2)^2+trrt(1,1)^2+trrt(1,3)^2))*180/pi; %Goc xoay
ry = acos(trrt(2,2)/sqrt(trrt(2,2)^2+trrt(2,1)^2+trrt(2,3)^2))*180/pi;
rz = acos(trrt(3,3)/sqrt(trrt(3,2)^2+trrt(3,1)^2+trrt(3,3)^2))*180/pi;

handles.rx.String = num2str(round(rx,5));
handles.ry.String = num2str(round(ry,5));
handles.rz.String = num2str(round(rz,5));


% handles.y.String = num2str(M(2,4));
% handles.z.String = num2str(M(3,1));

% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
axis on
px = str2double(handles.x.String);
py = str2double(handles.y.String);
pz = str2double(handles.z.String);
rrx = str2double(handles.rx.String);
rry = str2double(handles.ry.String);
rrz = str2double(handles.rz.String);
l1 = 251;
l2 = 225;
l3 = 150;
l5 = 60;
L(1) = Link([0  l1    0  pi/2],'standard');
L(2) = Link([0   0    l2    0],'standard');
L(3) = Link([0   0    l3    0],'standard');
L(4) = Link([0   0     0 pi/2],'standard');
L(5) = Link([0  l5     0    0],'standard');


Robot = SerialLink(L);
Robot.name = 'Robot';
nx = cos(rry)*cos(rrx);
ny = cos(rry)*sin(rrx);
nz = sin(rry);
ox = -sin(rrz)*sin(rry)*cos(rrx)-cos(rrz)*sin(rrx);
oy = -sin(rrz)*sin(rry)*sin(rrx)+cos(rrz)*cos(rrx);
oz = sin(rrz)*cos(rry);
ax = sin(rrz)*sin(rrx)- cos(rrz)*sin(rry)*cos(rrx);
ay = -sin(rrz)*cos(rrx)- cos(rrz)*sin(rry)*sin(rrx);
az = -cos(rrz)*cos(rry);
M = [ nx ox ax px;
      ny oy ay py;
      nz oz az pz;
      0   0  0  1];
J = Robot.ikine(M,[0 0 0 0 0],'mask', [1 1 1 0 0 0])*180/pi;


Robot.plot(J*pi/180)

axis([-700 700 -700 700 -700 700])
handles.theta1_.String = num2str(round(J(1),3));

handles.theta2_.String = num2str(round(J(2),3));

handles.theta3_.String = num2str(round(J(3),3));

handles.theta4_.String = num2str(round(J(4),3));

handles.theta5_.String = num2str(round(J(5),3));





% --- Executes on button press in pushbutton3.
function pushbutton3_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
l1 = 251;
l2 = 225;
l3 = 150;
l5 = 60;
L(1) = Link([0  l1    0  pi/2],'standard');
L(2) = Link([0   0    l2    0],'standard');
L(3) = Link([0   0    l3    0],'standard');
L(4) = Link([0   0     0 pi/2],'standard');
L(5) = Link([0  l5     0    0],'standard');


Robot = SerialLink(L);
Robot.name = 'Robot';
handles.theta1_.String = num2str(0);

handles.theta2_.String = num2str(0);

handles.theta3_.String = num2str(0);

handles.theta4_.String = num2str(90);

handles.theta5_.String = num2str(90);
t1 =0;
t2 =0;
t3=0;
t4=pi/2;
t5 =0 ;
Robot.plot([t1 t2 t3 t4 t5]);
axis([-700 700 -700 700 -700 700]);

M = Robot.fkine([t1 t2 t3 t4 t5]);
T = M(1);
trsl = transl(T);
trrt = tr2rt(T);
xx = trsl(1);
yy = trsl(2);
zz = trsl(3);

handles.x.String = num2str(round(xx,5));
handles.y.String = num2str(round(yy,5));
handles.z.String = num2str(round(zz,5));
rx = acos(trrt(1,1)/sqrt(trrt(1,2)^2+trrt(1,1)^2+trrt(1,3)^2))*180/pi; %Goc xoay
ry = acos(trrt(2,2)/sqrt(trrt(2,2)^2+trrt(2,1)^2+trrt(2,3)^2))*180/pi;
rz = acos(trrt(3,3)/sqrt(trrt(3,2)^2+trrt(3,1)^2+trrt(3,3)^2))*180/pi;

handles.rx.String = num2str(round(rx,5));
handles.ry.String = num2str(round(ry,5));
handles.rz.String = num2str(round(rz,5));





function y_Callback(hObject, eventdata, handles)
% hObject    handle to y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of y as text
%        str2double(get(hObject,'String')) returns contents of y as a double


% --- Executes during object creation, after setting all properties.
function y_CreateFcn(hObject, eventdata, handles)
% hObject    handle to y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function z_Callback(hObject, eventdata, handles)
% hObject    handle to z (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of z as text
%        str2double(get(hObject,'String')) returns contents of z as a double


% --- Executes during object creation, after setting all properties.
function z_CreateFcn(hObject, eventdata, handles)
% hObject    handle to z (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function rx_Callback(hObject, eventdata, handles)
% hObject    handle to rx (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of rx as text
%        str2double(get(hObject,'String')) returns contents of rx as a double


% --- Executes during object creation, after setting all properties.
function rx_CreateFcn(hObject, eventdata, handles)
% hObject    handle to rx (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function ry_Callback(hObject, eventdata, handles)
% hObject    handle to ry (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of ry as text
%        str2double(get(hObject,'String')) returns contents of ry as a double


% --- Executes during object creation, after setting all properties.
function ry_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ry (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function rz_Callback(hObject, eventdata, handles)
% hObject    handle to rz (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of rz as text
%        str2double(get(hObject,'String')) returns contents of rz as a double


% --- Executes during object creation, after setting all properties.
function rz_CreateFcn(hObject, eventdata, handles)
% hObject    handle to rz (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function theta1__Callback(hObject, eventdata, handles)
% hObject    handle to theta1_ (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of theta1_ as text
%        str2double(get(hObject,'String')) returns contents of theta1_ as a double


% --- Executes during object creation, after setting all properties.
function theta1__CreateFcn(hObject, eventdata, handles)
% hObject    handle to theta1_ (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function theta2__Callback(hObject, eventdata, handles)
% hObject    handle to theta2_ (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of theta2_ as text
%        str2double(get(hObject,'String')) returns contents of theta2_ as a double


% --- Executes during object creation, after setting all properties.
function theta2__CreateFcn(hObject, eventdata, handles)
% hObject    handle to theta2_ (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function theta3__Callback(hObject, eventdata, handles)
% hObject    handle to theta3_ (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of theta3_ as text
%        str2double(get(hObject,'String')) returns contents of theta3_ as a double


% --- Executes during object creation, after setting all properties.
function theta3__CreateFcn(hObject, eventdata, handles)
% hObject    handle to theta3_ (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function theta4__Callback(hObject, eventdata, handles)
% hObject    handle to theta4_ (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of theta4_ as text
%        str2double(get(hObject,'String')) returns contents of theta4_ as a double


% --- Executes during object creation, after setting all properties.
function theta4__CreateFcn(hObject, eventdata, handles)
% hObject    handle to theta4_ (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function theta5__Callback(hObject, eventdata, handles)
% hObject    handle to theta5_ (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of theta5_ as text
%        str2double(get(hObject,'String')) returns contents of theta5_ as a double


% --- Executes during object creation, after setting all properties.
function theta5__CreateFcn(hObject, eventdata, handles)
% hObject    handle to theta5_ (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function theta6__Callback(hObject, eventdata, handles)
% hObject    handle to theta6_ (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of theta6_ as text
%        str2double(get(hObject,'String')) returns contents of theta6_ as a double


% --- Executes during object creation, after setting all properties.
function theta6__CreateFcn(hObject, eventdata, handles)
% hObject    handle to theta6_ (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton4.
function pushbutton4_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
closereq();



function edit13_Callback(hObject, eventdata, handles)
% hObject    handle to edit13 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
clc;
diary('example')
diary on;
%COMMANDS YOU WANT TO BE SHOWN IN THE COMMAND WINDOW
disp(array);

diary off;
output=fileread('example');
%FINALLY
set(handles.edit13,'string',output);
delete('example');%OPTIONAL
% Hints: get(hObject,'String') returns contents of edit13 as text
%        str2double(get(hObject,'String')) returns contents of edit13 as a double


% --- Executes during object creation, after setting all properties.
function edit13_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit13 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
