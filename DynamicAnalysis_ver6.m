function varargout = DynamicAnalysis_ver6(varargin)
% DYNAMICANALYSIS_VER6 MATLAB code for DynamicAnalysis_ver6.fig
%      DYNAMICANALYSIS_VER6, by itself, creates a new DYNAMICANALYSIS_VER6 or raises the existing
%      singleton*.
%
%      H = DYNAMICANALYSIS_VER6 returns the handle to a new DYNAMICANALYSIS_VER6 or the handle to
%      the existing singleton*.
%
%      DYNAMICANALYSIS_VER6('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in DYNAMICANALYSIS_VER6.M with the given input arguments.
%
%      DYNAMICANALYSIS_VER6('Property','Value',...) creates a new DYNAMICANALYSIS_VER6 or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before DynamicAnalysis_ver6_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to DynamicAnalysis_ver6_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help DynamicAnalysis_ver6

% Last Modified by GUIDE v2.5 04-Jan-2023 15:50:14

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @DynamicAnalysis_ver6_OpeningFcn, ...
                   'gui_OutputFcn',  @DynamicAnalysis_ver6_OutputFcn, ...
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


% --- Executes during object creation, after setting all properties.
function robot_plot_CreateFcn(hObject, eventdata, handles)
% hObject    handle to robot_plot (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: place code in OpeningFcn to populate robot_plot
% --- Executes during object creation, after setting all properties.
function torque_CreateFcn(hObject, eventdata, handles)
% hObject    handle to torque (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% --- Executes during object creation, after setting all properties.
function speed_CreateFcn(hObject, eventdata, handles)
% hObject    handle to speed (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: place code in OpeningFcn to populate speed

% Hint: place code in OpeningFcn to populate torque
% --- Executes just before DynamicAnalysis_ver6 is made visible.
function DynamicAnalysis_ver6_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to DynamicAnalysis_ver6 (see VARARGIN)

% Choose default command line output for DynamicAnalysis_ver6
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);
clc;

global Robot
global grav
global fext
l1 = 0.251;
l2 = 0.225;
l3 = 0.15;
l5 = 0.06;
L(1) = Link([0  l1    0  pi/2],'standard');
L(2) = Link([0   0    l2    0],'standard');
L(3) = Link([0   0    l3    0],'standard');
L(4) = Link([0   0     0 pi/2],'standard');
L(5) = Link([0  l5     0    0],'standard');


Robot = SerialLink(L);
Robot.name = 'Robot';
grav = [0;0;9.81];
fext = [1 1 1 1 1 1];
for i=1:1:5

Robot.links(i).r = [0 0 1];

Robot.links(i).Jm = 1;
end 
Robot.links(1).m = 1;
Robot.links(2).m = 1;
Robot.links(3).m = 1;
Robot.links(4).m = 1;
Robot.links(5).m = 1;
Robot.links(1).I = [1 1 1;
                    1 1 1;
                    1 1 1];
Robot.links(2).I = [1 1 1;
                    1 1 1;
                    1 1 1];
Robot.links(3).I = [1 1 1;
                    1 1 1;
                    1 1 1];


Robot.links(4).I = [1 1 1;
                    1 1 1;
                    1 1 1];
Robot.links(5).I = [1 1 1;
                    1 1 1;
                    1 1 1];
grav = [0;0;9.81];
fext = [1 1 1 1 1 1];
axes(handles.robot_plot);
Robot.plot([0 0 0 pi/2 0]);
axes(handles.torque);
grid on;
xlabel('t[s]')
ylabel('\tau(NM)')
axes(handles.speed);
grid on;
xlabel('t[s]')
ylabel('speed(RPM)')


% UIWAIT makes DynamicAnalysis_ver6 wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = DynamicAnalysis_ver6_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in analyze.
function analyze_Callback(hObject, eventdata, handles)
% hObject    handle to analyze (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global Robot
global qd 
global tau
global grav
global fext
l1 = 0.251;
l2 = 0.225;
l3 = 0.15;
l5 = 0.06;

cor = [];
% L =  l2+l3+l5;
%% CIRCLE
% des = 2*pi;
% step = 0.1;
% for i=0:step:des
%     cor= [cor;[sin(i)*0.1+0.2 cos(i)*0.1+0 0.28]]; 
% end
%% LINE
% des = 0.5;
% step = 0.01;
% for i=0:step:des
%     cor= [cor;[i-0.25 0.25+0 0.28]]; 
% end
%% HELIX
% des = 7*pi;
% step = 0.2;
% for i=0:step:des
%     cor= [cor;[sin(i)*0.1+0.2 cos(i)*0.1+0 0.02*i]]; 
% end

%% HEART
% step = 3;
% des = 60;
% for i=0:step:des
%     cor= [cor;[0.0001*(-i^2+40*i+1200)*sin(pi*i/180)+0.1 0.0001*(-i^2+40*i+1200)*cos(pi*i/180)+0.1 0.28]]; 
% end
% for i=des:-step:0
%     cor= [cor;[-0.0001*(-i^2+40*i+1200)*sin(pi*i/180)+0.1 0.0001*(-i^2+40*i+1200)*cos(pi*i/180)+0.1 0.28]]; 
% end

%% FLOWER
des = 2*pi;
step = 0.1;
for i=0:step:des
    cor= [cor;[sin(5*i)*cos(i)*0.1+0.2 sin(5*i)*sin(i)*0.1+0 0.28]]; 
end






% for i=0:step:des
%     cor= [cor;[sin(i)*cos(i)*log(abs(i)) sqrt(abs(i))*cos(i) 0.28]]; 
% end
% for i=0:step:des
%     cor= [cor;[0.01*(-i^2+40*i+1200)*sin(pi*i/180)+0.02 0.01*(-i^2+40*i+1200)*cos(pi*i/180)+0.02 0.28]]; 
% end


theta1 =sign(cor(:,2)).*acos(cor(:,1)./sqrt(cor(:,1).^2+cor(:,2).^2));
px = sqrt(cor(:,1).^2+cor(:,2).^2);
py = cor(:,3)-l1+l5;
c3 = (px.^2 + py.^2 -l2^2 - l3^2)/(2*l2*l3);
s3 = -sqrt(abs(1-c3.^2));
t3 = atan2(s3,c3);

s2 = py.*(l2 + l3*c3) - px*l3.*s3;
c2 = px.*(l2+l3*c3) + py*l3.*s3;
% c2 = (px*(l2+l3*c3)+py*l3*s3)/((l2+l3*c3)^2+(l3*s3)^2);
% s2 =  (py*(l2+l3*c3)+px*l3*s3)/((l2+l3*c3)^2+(l3*s3)^2);
t2 = atan2(s2,c2);
theta2 = t2;
theta3 =t3;
theta4=2*pi-theta2-theta3;
theta5=-theta1;
Q(:,1)=theta1;
Q(:,2)=theta2;
Q(:,3)=theta3;
Q(:,4)=theta4;
Q(:,5)=theta5;
TRAJ = fkine(Robot,Q);


for i = 1:1:length(cor)
    T = TRAJ(i);
    trsl = transl(T);
    xx(i) = trsl(1);
    yy(i) = trsl(2);
    zz(i) = trsl(3);

end
qd = [];
for i=1:5
qd = [qd,diff(Q(:,i))];
end
qdd = [];
for i=1:5
qdd = [qdd,diff(qd(:,i))];
end
q = Q(1:length(qdd),1:5);
qd = qd(1:length(qdd),1:5);

tau = rne_dh(Robot, q, qd, qdd,grav,fext);
t = 1:length(qd);
for i = 1:1:length(q)
%    axis([-0.7 0.7 -0.7 0.7 -0.7 0.7])
%    set(gca, 'DataAspectRatio',[1 1 1])
  
   axes(handles.robot_plot);
   Robot.plot(Q(i,:))
   hold on
   plot3(xx(i),yy(i),zz(i),'ro','LineWidth',1,'MarkerSize',3);
   hold on
   axes(handles.torque);
   
   plot(t(1:i)-1,tau(1:i,1),'r',t(1:i)-1,tau(1:i,2),'k',t(1:i)-1,tau(1:i,3),'b',t(1:i)-1,tau(1:i,4),'g',t(1:i)-1,tau(1:i,5),'c','LineWidth',2);
   grid on;
   xlabel('t[s]')
   ylabel('\tau(NM)')
   axes(handles.speed);
    plot(t(1:i)-1,qd(1:i,1)*0.1047,'r',t(1:i)-1,qd(1:i,2)*0.1047,'k',t(1:i)-1,qd(1:i,3)*0.1047,'b',t(1:i)-1,qd(1:i,4)*0.1047,'g',t(1:i)-1,qd(1:i,5)*0.1047,'c','LineWidth',2);
   grid on;
   xlabel('t[s]')
   ylabel('speed(RPM)')
end



% --- Executes on button press in export.
function export_Callback(hObject, eventdata, handles)
% hObject    handle to export (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global qd
global tau
figure(1)
t = 0:length(qd)-1;
subplot(3,2,1)
plot(t,tau(:,1),'r')
grid on
xlabel('t[s]')
ylabel('\tau(NM)')
title('Joint Torque 1 (max: '+string(max(tau(:,1)))+'Nm, min: '+ string(min(tau(:,1)))+'Nm)', 'FontSize', 14);

subplot(3,2,2)
plot(t,tau(:,2),'k')
grid on
xlabel('t[s]')
ylabel('\tau(NM)')
title('Joint Torque 2 (max: '+string(max(tau(:,2)))+'Nm, min: '+ string(min(tau(:,2)))+'Nm)', 'FontSize', 14);

subplot(3,2,3)
plot(t,tau(:,3),'b')
grid on
xlabel('t[s]')
ylabel('\tau(NM)')
title('Joint Torque 3 (max: '+string(max(tau(:,3)))+'Nm, min: '+ string(min(tau(:,3)))+'Nm)', 'FontSize', 14);


subplot(3,2,4)
plot(t,tau(:,4),'g')
grid on
xlabel('t[s]')
ylabel('\tau(NM)')
title('Joint Torque 4 (max: '+string(max(tau(:,4)))+'Nm, min: '+ string(min(tau(:,4)))+'Nm)', 'FontSize', 14);

subplot(3,2,5)
plot(t,tau(:,5),'c')
grid on
xlabel('t[s]')
ylabel('\tau(NM)')
title('Joint Torque 5 (max: '+string(max(tau(:,5)))+'Nm, min: '+ string(min(tau(:,5)))+'Nm)', 'FontSize', 14);

subplot(3,2,6)
plot(t,tau(:,1),'r',t,tau(:,2),'k',t,tau(:,3),'b',t,tau(:,4),'g',t,tau(:,5),'c')
grid on
xlabel('t[s]')
ylabel('\tau(NM)')




%% RPM
figure(2)

subplot(3,2,1)
plot(t,qd(:,1)*0.1047,'r')
grid on
xlabel('t[s]')
ylabel('RPM')
title('Joint1 (max: '+string(max(qd(:,1)*0.1047))+'rpm, mean: '+ string(mean(qd(:,1)*0.1047))+'rpm)', 'FontSize', 14);

subplot(3,2,2)
plot(t,qd(:,2)*0.1047,'k')
grid on
xlabel('t[s]')
ylabel('RPM')
title('Joint2 (max: '+string(max(qd(:,2)*0.1047))+'rpm, mean: '+ string(mean(qd(:,2)*0.1047))+'rpm)', 'FontSize', 14);

subplot(3,2,3)
plot(t,qd(:,3)*0.1047,'b')
grid on
xlabel('t[s]')
ylabel('RPM')
title('Joint3 (max: '+string(max(qd(:,3)*0.1047))+'rpm, mean: '+ string(mean(qd(:,3)*0.1047))+'rpm)', 'FontSize', 14);


subplot(3,2,4)
plot(t,qd(:,4)*0.1047,'g')
grid on
xlabel('t[s]')
ylabel('RPM')
title('Joint 4 (max: '+string(max(qd(:,4)*0.1047))+'rpm, mean: '+ string(mean(qd(:,4)*0.1047))+'rpm)', 'FontSize', 14);

subplot(3,2,5)
plot(t,qd(:,5)*0.1047,'c')
grid on
xlabel('t[s]')
ylabel('RPM')
title('Joint 5 (max: '+string(max(qd(:,5)*0.1047))+'rpm, mean: '+ string(mean(qd(:,5)*0.1047))+'rpm)', 'FontSize', 14);

subplot(3,2,6)
plot(t,qd(:,1)*0.1047,'r',t,qd(:,2)*0.1047,'k',t,qd(:,3)*0.1047,'b',t,qd(:,4)*0.1047,'g',t,qd(:,5)*0.1047,'c')
grid on
xlabel('t[s]')
ylabel('RPM')




% --- Executes on button press in exit.
function exit_Callback(hObject, eventdata, handles)
% hObject    handle to exit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
closereq(); 
