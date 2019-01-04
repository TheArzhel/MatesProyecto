function varargout = trackBall(varargin)
% TRACKBALL MATLAB code for trackBall.fig
%      TRACKBALL, by itself, creates a new TRACKBALL or raises the existing
%      singleton*.
%
%      H = TRACKBALL returns the handle to a new TRACKBALL or the handle to
%      the existing singleton*.
%
%      TRACKBALL('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in TRACKBALL.M with the given input arguments.
%
%      TRACKBALL('Property','Value',...) creates a new TRACKBALL or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before trackBall_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to trackBall_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help trackBall

% Last Modified by GUIDE v2.5 24-Nov-2016 11:52:15

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @trackBall_OpeningFcn, ...
                   'gui_OutputFcn',  @trackBall_OutputFcn, ...
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


% --- Executes just before trackBall is made visible.
function trackBall_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to trackBall (see VARARGIN)


set(hObject,'WindowButtonDownFcn',{@my_MouseClickFcn,handles.axes1});
set(hObject,'WindowButtonUpFcn',{@my_MouseReleaseFcn,handles.axes1});
axes(handles.axes1);

handles.Cube=DrawCube(eye(3));

set(handles.axes1,'CameraPosition',...
    [0 0 5],'CameraTarget',...
    [0 0 -5],'CameraUpVector',...
    [0 1 0],'DataAspectRatio',...
    [1 1 1]);

set(handles.axes1,'xlim',[-3 3],'ylim',[-3 3],'visible','off','color','none');

% Choose default command line output for trackBall
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes trackBall wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = trackBall_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

function my_MouseClickFcn(obj,event,hObject)

handles=guidata(obj);
xlim = get(handles.axes1,'xlim');
ylim = get(handles.axes1,'ylim');
mousepos=get(handles.axes1,'CurrentPoint');
xmouse = mousepos(1,1);
ymouse = mousepos(1,2);

if xmouse > xlim(1) && xmouse < xlim(2) && ymouse > ylim(1) && ymouse < ylim(2)
    Set_init_vec(RotVec(3,xmouse,ymouse))
    set(handles.figure1,'WindowButtonMotionFcn',{@my_MouseMoveFcn,hObject});
end
guidata(hObject,handles)

function my_MouseReleaseFcn(obj,event,hObject)
handles=guidata(hObject);
set(handles.figure1,'WindowButtonMotionFcn','');
guidata(hObject,handles);

function my_MouseMoveFcn(obj,event,hObject)

handles=guidata(obj);
xlim = get(handles.axes1,'xlim');
ylim = get(handles.axes1,'ylim');
mousepos=get(handles.axes1,'CurrentPoint');
xmouse = mousepos(1,1);
ymouse = mousepos(1,2);

if xmouse > xlim(1) && xmouse < xlim(2) && ymouse > ylim(1) && ymouse < ylim(2)

    
Init_Vec=Get_init_vec();
Axis=-cross(RotVec(3,xmouse,ymouse), Init_Vec);


Angle = acosd((RotVec(3,xmouse,ymouse)'*Init_Vec)/(norm(RotVec(3,xmouse,ymouse))*norm(Init_Vec)))*0.2;

RotMat=axisangle2matrix(Axis,Angle);

      %Writing quaternions
     if(isempty(GetLastQuaternion()))
        Quat0 = [0;0;0;0];
    else
         Quat0 = GetLastQuaternion();
     end
    
    set(handles.Quat0_0,'String',Quat0(1,1));
    set(handles.Quat0_1,'String',Quat0(2,1));
    set(handles.Quat0_2,'String',Quat0(3,1));
    set(handles.Quat0_3,'String',Quat0(4,1));
    
   Quat1 = TwoVec_To_Quat(Init_Vec,RotVec(3,xmouse,ymouse));
    set(handles.Quat1_0,'String',Quat1(1,1));
    set(handles.Quat1_1,'String',Quat1(2,1));
    set(handles.Quat1_2,'String',Quat1(3,1));
    set(handles.Quat1_3,'String',Quat1(4,1));

      Set_last_quaternion(Quat1);
    
    
    Quat_Product=quatmult(Quat0,Quat1);
    set(handles.Quat_Product_0,'String',Quat_Product(1,1));
    set(handles.Quat_Product_1,'String',Quat_Product(2,1));
    set(handles.Quat_Product_2,'String',Quat_Product(3,1));
    set(handles.Quat_Product_3,'String',Quat_Product(4,1));
      
    
     %Write Euler axis & angle
    [Euler_Axis,Euler_Angle]=RotMatToEulerAxis_Angle(RotMat);
    set(handles.Euler_AxisX,'String',Euler_Axis(1,1));
    set(handles.Euler_AxisY,'String',Euler_Axis(2,1));
    set(handles.Euler_AxisZ,'String',Euler_Axis(3,1));
    set(handles.Euler_Angle,'String',Euler_Angle);
    
    
    %Write Euler Angles
    [Pitch,Roll,Yaw]=RotMatToEulerAngles(RotMat);
    set(handles.Pitch,'String',Pitch);
    set(handles.Roll,'String',Roll);
    set(handles.Yaw,'String',Yaw);

    %Write Rot vector
    [Euler_Axis,Euler_Angle]=RotMatToEulerAxis_Angle(RotMat);
    Rot_Vec = Obt_RotVec(Euler_Angle,Euler_Axis);
    set(handles.x,'String',Rot_Vec(1));
    set(handles.y,'String',Rot_Vec(2));
    set(handles.z,'String',Rot_Vec(3));

     
    %Write RotMat 
    Set_rotation_matrix(RotMat);
    
    set(handles.RotMat_Pos1_1,'String',RotMat(1,1));
    set(handles.RotMat_Pos1_2,'String',RotMat(1,2));
    set(handles.RotMat_Pos1_3,'String',RotMat(1,3));

    set(handles.RotMat_Pos2_1,'String',RotMat(2,1));
    set(handles.RotMat_Pos2_2,'String',RotMat(2,2));
    set(handles.RotMat_Pos2_3,'String',RotMat(2,3));

    set(handles.RotMat_Pos3_1,'String',RotMat(3,1));
    set(handles.RotMat_Pos3_2,'String',RotMat(3,2));
    set(handles.RotMat_Pos3_3,'String',RotMat(3,3));
    
    
   
    handles.Cube = RedrawCube(RotMat,handles.Cube);
    
end
guidata(hObject,handles);

function h = DrawCube(R)

M0 = [    -1  -1 1;   %Node 1
    -1   1 1;   %Node 2
    1   1 1;   %Node 3
    1  -1 1;   %Node 4
    -1  -1 -1;  %Node 5
    -1   1 -1;  %Node 6
    1   1 -1;  %Node 7
    1  -1 -1]; %Node 8

M = (R*M0')';


x = M(:,1);
y = M(:,2);
z = M(:,3);


con = [1 2 3 4;
    5 6 7 8;
    4 3 7 8;
    1 2 6 5;
    1 4 8 5;
    2 3 7 6]';

x = reshape(x(con(:)),[4,6]);
y = reshape(y(con(:)),[4,6]);
z = reshape(z(con(:)),[4,6]);

c = 1/255*[255 248 88;
    0 0 0;
    57 183 225;
    57 183 0;
    255 178 0;
    255 0 0];

h = fill3(x,y,z, 1:6);

for q = 1:length(c)
    h(q).FaceColor = c(q,:);
end

function h = RedrawCube(R,hin)

h = hin;
c = 1/255*[255 248 88;
    0 0 0;
    57 183 225;
    57 183 0;
    255 178 0;
    255 0 0];

M0 = [    -1  -1 1;   %Node 1
    -1   1 1;   %Node 2
    1   1 1;   %Node 3
    1  -1 1;   %Node 4
    -1  -1 -1;  %Node 5
    -1   1 -1;  %Node 6
    1   1 -1;  %Node 7
    1  -1 -1]; %Node 8

M = (R*M0')';


x = M(:,1);
y = M(:,2);
z = M(:,3);


con = [1 2 3 4;
    5 6 7 8;
    4 3 7 8;
    1 2 6 5;
    1 4 8 5;
    2 3 7 6]';

x = reshape(x(con(:)),[4,6]);
y = reshape(y(con(:)),[4,6]);
z = reshape(z(con(:)),[4,6]);

for q = 1:6
    h(q).Vertices = [x(:,q) y(:,q) z(:,q)];
    h(q).FaceColor = c(q,:);
end

%---------------------------------------------------------------------------
function Set_init_vec(vec)
global initial_vector;
initial_vector = vec;

function Set_rotation_matrix(RotMat)
global rotation_mat;
rotation_mat = RotMat;

function Set_last_quaternion(Quat)
global last_quat;
last_quat = Quat;

function Set_last_cube(Cube)
global last_cube;
last_cube = Cube;

function vec = Get_init_vec();
global initial_vector;
vec = initial_vector;

function Mat = Get_rotation_matrix();
global rotation_mat;
Mat = rotation_mat;

function Quat = GetLastQuaternion();
global last_quat;
Quat = last_quat;

function Cube = GetLastCube();
global last_cube;
Cube = last_cube;

%---------------------------------------------------------------------------

function Quat0_0_Callback(hObject, eventdata, handles)
% hObject    handle to q0_0 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of q0_0 as text
%        str2double(get(hObject,'String')) returns contents of q0_0 as a double


% --- Executes during object creation, after setting all properties.
function Quat0_0_CreateFcn(hObject, eventdata, handles)
% hObject    handle to q0_0 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Quat0_1_Callback(hObject, eventdata, handles)
% hObject    handle to q0_1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of q0_1 as text
%        str2double(get(hObject,'String')) returns contents of q0_1 as a double


% --- Executes during object creation, after setting all properties.
function Quat0_1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to q0_1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Quat0_2_Callback(hObject, eventdata, handles)
% hObject    handle to q0_2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of q0_2 as text
%        str2double(get(hObject,'String')) returns contents of q0_2 as a double


% --- Executes during object creation, after setting all properties.
function Quat0_2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to q0_2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Quat0_3_Callback(hObject, eventdata, handles)
% hObject    handle to q0_3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of q0_3 as text
%        str2double(get(hObject,'String')) returns contents of q0_3 as a double


% --- Executes during object creation, after setting all properties.
function Quat0_3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to q0_3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Quat1_0_Callback(hObject, eventdata, handles)
% hObject    handle to q1_0 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of q1_0 as text
%        str2double(get(hObject,'String')) returns contents of q1_0 as a double


% --- Executes during object creation, after setting all properties.
function Quat1_0_CreateFcn(hObject, eventdata, handles)
% hObject    handle to q1_0 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Quat1_1_Callback(hObject, eventdata, handles)
% hObject    handle to q1_1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of q1_1 as text
%        str2double(get(hObject,'String')) returns contents of q1_1 as a double


% --- Executes during object creation, after setting all properties.
function Quat1_1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to q1_1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Quat1_2_Callback(hObject, eventdata, handles)
% hObject    handle to q1_2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of q1_2 as text
%        str2double(get(hObject,'String')) returns contents of q1_2 as a double


% --- Executes during object creation, after setting all properties.
function Quat1_2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to q1_2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Quat1_3_Callback(hObject, eventdata, handles)
% hObject    handle to q1_3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of q1_3 as text
%        str2double(get(hObject,'String')) returns contents of q1_3 as a double


% --- Executes during object creation, after setting all properties.
function Quat1_3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to q1_3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function QuatProduct_0_Callback(hObject, eventdata, handles)
% hObject    handle to qk_0 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hints: get(hObject,'String') returns contents of qk_0 as text
%        str2double(get(hObject,'String')) returns contents of qk_0 as a double

% --- Executes during object creation, after setting all properties.
function QuatProduct_0_CreateFcn(hObject, eventdata, handles)
% hObject    handle to qk_0 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function QuatProduct_1_Callback(hObject, eventdata, handles)
% hObject    handle to qk_1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hints: get(hObject,'String') returns contents of qk_1 as text
%        str2double(get(hObject,'String')) returns contents of qk_1 as a double

% --- Executes during object creation, after setting all properties.
function QuatProduct_1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to qk_1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function QuatProduct_2_Callback(hObject, eventdata, handles)
% hObject    handle to qk_2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hints: get(hObject,'String') returns contents of qk_2 as text
%        str2double(get(hObject,'String')) returns contents of qk_2 as a double

% --- Executes during object creation, after setting all properties.
function QuatProduct_2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to qk_2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function QuatProduct_3_Callback(hObject, eventdata, handles)
% hObject    handle to qk_3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hints: get(hObject,'String') returns contents of qk_3 as text
%        str2double(get(hObject,'String')) returns contents of qk_3 as a double

% --- Executes during object creation, after setting all properties.
function QuatProduct_3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to qk_3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function Euler_Axisx_Callback(hObject, eventdata, handles)
% hObject    handle to e_axis_x (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hints: get(hObject,'String') returns contents of e_axis_x as text
%        str2double(get(hObject,'String')) returns contents of e_axis_x as a double

% --- Executes during object creation, after setting all properties.
function Euler_Axisx_CreateFcn(hObject, eventdata, handles)
% hObject    handle to e_axis_x (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function Euler_Axisy_Callback(hObject, eventdata, handles)
% hObject    handle to e_axis_y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hints: get(hObject,'String') returns contents of e_axis_y as text
%        str2double(get(hObject,'String')) returns contents of e_axis_y as a double

% --- Executes during object creation, after setting all properties.
function Euler_Axisy_CreateFcn(hObject, eventdata, handles)
% hObject    handle to e_axis_y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function Euler_Axisz_Callback(hObject, eventdata, handles)
% hObject    handle to e_axis_z (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of e_axis_z as text
%        str2double(get(hObject,'String')) returns contents of e_axis_z as a double

% --- Executes during object creation, after setting all properties.
function Euler_Axisz_CreateFcn(hObject, eventdata, handles)
% hObject    handle to e_axis_z (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function Euler_Angle_Callback(hObject, eventdata, handles)
% hObject    handle to e_angle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hints: get(hObject,'String') returns contents of e_angle as text
%        str2double(get(hObject,'String')) returns contents of e_angle as a double

% --- Executes during object creation, after setting all properties.
function Euler_Angle_CreateFcn(hObject, eventdata, handles)
% hObject    handle to e_angle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function phi_Callback(hObject, eventdata, handles)
% hObject    handle to phi (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hints: get(hObject,'String') returns contents of phi as text
%        str2double(get(hObject,'String')) returns contents of phi as a double

% --- Executes during object creation, after setting all properties.
function phi_CreateFcn(hObject, eventdata, handles)
% hObject    handle to phi (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function theta_Callback(hObject, eventdata, handles)
% hObject    handle to theta (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hints: get(hObject,'String') returns contents of theta as text
%        str2double(get(hObject,'String')) returns contents of theta as a double

% --- Executes during object creation, after setting all properties.
function theta_CreateFcn(hObject, eventdata, handles)
% hObject    handle to theta (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function psi_Callback(hObject, eventdata, handles)
% hObject    handle to psi (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hints: get(hObject,'String') returns contents of psi as text
%        str2double(get(hObject,'String')) returns contents of psi as a double

% --- Executes during object creation, after setting all properties.
function psi_CreateFcn(hObject, eventdata, handles)
% hObject    handle to psi (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
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
function pusheulerangles_Callback(hObject, eventdata, handles)
% hObject    handle to pusheulerangles (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
Set_last_cube(handles.Cube);

Pitch = str2double(get(handles.Pitch,'String'));
Roll = str2double(get(handles.Roll,'String'));
Yaw = str2double(get(handles.Yaw,'String'));

RotMat=EulerAnglesToRotMat(Pitch,Roll,Yaw);

%---------------------------------------------------------------------------------
    
    %Write quaternions
     if(isempty(GetLastQuaternion()))
        Quat0 = [0;0;0;0];
    else
        Quat0 = GetLastQuaternion();
    end
    set(handles.Quat_0,'String',Quat0(1,1));
    set(handles.Quat_1,'String',Quat0(2,1));
    set(handles.Quat_2,'String',Quat0(3,1));
    set(handles.Quat_3,'String',Quat0(4,1));

    Quat1 = EulerAngle_to_Quat(Pitch,Roll,Yaw);
    set(handles.Quat1_0,'String',Quat1(1,1));
    set(handles.Quat1_1,'String',Quat1(2,1));
    set(handles.Quat1_2,'String',Quat1(3,1));
    set(handles.Quat1_3,'String',Quat1(4,1));

    Set_last_quaternion(Quat1);
    
    Quat_Product = quatmult(Quat0,Quat1);
    set(handles.Quat_Product_0,'String',Quat_Product(1,1));
    set(handles.Quat_Product_1,'String',Quat_Product(2,1));
    set(handles.Quat_Product_2,'String',Quat_Product(3,1));
    set(handles.Quat_Product_3,'String',Quat_Product(4,1));
    
    
     %Write Euler axis & angle
    [Euler_Axis,Euler_Angle]=RotMatToEulerAngles(RotMat);
    set(handles.Euler_AxisX,'String',Euler_Axis(1,1));
    set(handles.Euler_AxisY,'String',Euler_Axis(2,1));
    set(handles.Euler_AxisZ,'String',Euler_Axis(3,1));
    set(handles.Euler_Angle,'String',Euler_Angle);
    
    %Write rotation vector
    [Euler_Axis,Euler_Angle]=rotMat2Eaa(RotMat);
    Rot_Vec = Obt_RotVec(Euler_Axis,Euler_Angle);%Check
    set(handles.x,'String',Rot_Vec(1));
    set(handles.y,'String',Rot_Vec(2));
    set(handles.z,'String',Rot_Vec(3));
    
     %Write RotMat 
    Set_rotation_matrix(RotMat);
    
    set(handles.RotMat_Pos1_1,'String',RotMat(1,1));
    set(handles.RotMat_Pos1_2,'String',RotMat(1,2));
    set(handles.RotMat_Pos1_3,'String',RotMat(1,3));

    set(handles.RotMat_Pos2_1,'String',RotMat(2,1));
    set(handles.RotMat_Pos2_2,'String',RotMat(2,2));
    set(handles.RotMat_Pos2_3,'String',RotMat(2,3));

    set(handles.RotMat_Pos3_1,'String',RotMat(3,1));
    set(handles.RotMat_Pos3_2,'String',RotMat(3,2));
    set(handles.RotMat_Pos3_3,'String',RotMat(3,3));
    
    last_cube = GetLastCube();
    handles.Cube = RedrawCube(RotMat,last_cube);
    Set_last_cube(handles.Cube);
    
    % --- Executes on button press in reset.
function reset_Callback(hObject, eventdata, handles)
% hObject    handle to reset (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.Cube = RedrawCube(eye(3,3) ,handles.Cube); 
set(handles.Quat0_0,'String',0);
set(handles.Quat0_1,'String',0);
set(handles.Quat0_2,'String',0);
set(handles.Quat0_3,'String',0);

set(handles.Quat1_0,'String',0);
set(handles.Quat1_1,'String',0);
set(handles.Quat1_2,'String',0);
set(handles.Quat1_3,'String',0);

Set_last_quaternion([0;0;0;0]);

set(handles.Quat_Product_0,'String',0);
set(handles.Quat_Product_1,'String',0);
set(handles.Quat_Product_2,'String',0);
set(handles.Quat_Product_3,'String',0);

set(handles.Euler_AxisX,'String',0);
set(handles.Euler_AxisY,'String',0);
set(handles.Euler_AxisZ,'String',0);
set(handles.Euler_Angle,'String',0);

set(handles.Pitch,'String',0);
set(handles.Roll,'String',0);
set(handles.Yaw,'String',0);

set(handles.x,'String',0);
set(handles.y,'String',0);
set(handles.z,'String',0);

set(handles.RotMat_Pos1_1,'String',1);
set(handles.RotMat_Pos1_2,'String',0);
set(handles.RotMat_Pos1_3,'String',0);

set(handles.RotMat_Pos2_1,'String',0);
set(handles.RotMat_Pos2_2,'String',1);
set(handles.RotMat_Pos2_3,'String',0);

set(handles.RotMat_Pos3_1,'String',0);
set(handles.RotMat_Pos3_2,'String',0);
set(handles.RotMat_Pos3_3,'String',1);
    
    % --- Executes during object creation, after setting all properties.
function axes1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to axes1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: place code in OpeningFcn to populate axes1

% --- Executes on mouse press over axes background.
function axes1_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to axes1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    
% --- Executes on button press in pusheuleraxis.
function pusheuleraxis_Callback(hObject, eventdata, handles)
% hObject    handle to pusheuleraxis (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
Set_last_cube(handles.Cube);

Euler_Angle = str2double(get(handles.Euler_Angle,'String'));
Euler_AxisX = str2double(get(handles.Euler_AxisX,'String'));
Euler_AxisY = str2double(get(handles.Euler_AxisY,'String'));
Euler_AxisZ = str2double(get(handles.Euler_AxisZ,'String'));

RotMat=axisangle2matrix([Euler_AxisX;Euler_AxisY;Euler_AxisZ],Euler_Angle);

 %Write Euler Angles
    [Pitch,Roll,Yaw]=RotMatToEulerAngles(RotMat);
    set(handles.Pitch,'String',Pitch);
    set(handles.Roll,'String',Roll);
    set(handles.Yaw,'String',Yaw);   

  %Write quaternions
    if(isempty(GetLastQuaternion()))
        Quat0 = [0;0;0;0];
    else
        Quat0 = GetLastQuaternion();
    end
    set(handles.Quat0_0,'String', Quat0(1,1));
    set(handles.Quat0_1,'String', Quat0(2,1));
    set(handles.Quat0_2,'String', Quat0(3,1));
    set(handles.Quat0_3,'String', Quat0(4,1));

    Quat1 = EulerAngle_to_Quat(Pitch,Roll,Yaw);
    set(handles.Quat1_0,'String', Quat1(1,1));
    set(handles.Quat1_1,'String', Quat1(2,1));
    set(handles.Quat1_2,'String', Quat1(3,1));
    set(handles.Quat1_3,'String', Quat1(4,1));

    Set_last_quaternion(Quat1);
    
   Quat_Product = quatmult( Quat0, Quat1);
    set(handles.Quat_Product_0,'String',Quat_Product(1,1));
    set(handles.Quat_Product_1,'String',Quat_Product(2,1));
    set(handles.Quat_Product_2,'String',Quat_Product(3,1));
    set(handles.Quat_Product_3,'String',Quat_Product(4,1));

    %Write rotation vector
    [Euler_Axis,Euler_Angle]=RotMatToEulerAxis_Angle(RotMat);
     Rot_Vec = Obt_RotVec(Euler_Axis,Euler_Angle);
    set(handles.x,'String',Rot_Vec(1));
    set(handles.y,'String',Rot_Vec(2));
    set(handles.z,'String',Rot_Vec(3));
    
     %Write r_mat 
    Set_rotation_matrix(RotMat);
    
    set(handles.RotMat_Pos1_1,'String',RotMat(1,1));
    set(handles.RotMat_Pos1_2,'String',RotMat(1,2));
    set(handles.RotMat_Pos1_3,'String',RotMat(1,3));

    set(handles.RotMat_Pos2_1,'String',RotMat(2,1));
    set(handles.RotMat_Pos2_2,'String',RotMat(2,2));
    set(handles.RotMat_Pos2_3,'String',RotMat(2,3));

    set(handles.RotMat_Pos3_1,'String',RotMat(3,1));
    set(handles.RotMat_Pos3_2,'String',RotMat(3,2));
    set(handles.RotMat_Pos3_3,'String',RotMat(3,3));
    
    last_cube = GetLastCube();
    handles.Cube = RedrawCube(RotMat,last_cube);
    Set_last_cube(handles.Cube);

function pushquaternions_Callback(hObject, eventdata, handles)
% hObject    handle to pushquaternions (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
Set_last_cube(handles.Cube);

Quat1(1,1) = str2double(get(handles.Quat1_0,'String'));
Quat1(2,1) = str2double(get(handles.Quat1_1,'String'));
Quat1(3,1) = str2double(get(handles.Quat1_2,'String'));
Quat1(4,1) = str2double(get(handles.Quat1_3,'String'));

[Pitch_AUX,Roll_AUX,Yaw_AUX]=Quat_To_EulerAngles(Quat1);
RotMat=EulerAnglesToRotMat(Pitch_AUX,Roll_AUX,Yaw_AUX);
%---------------------------Do All Again------------------------
   

    %Write Euler Angles
    [Pitch,Roll,Yaw]=RotMatToEulerAngles(RotMat);
    set(handles.Pitch,'String',Pitch);
    set(handles.Roll,'String',Roll);
    set(handles.Yaw,'String',Yaw);    

    %Write quaternions
    if(isempty(GetLastQuaternion()))
        Quat0 = [0;0;0;0];
    else
         Quat0 = GetLastQuaternion();
    end
     Quat1 = EulerAngle_to_Quat(Pitch,Roll,Yaw);
    Set_last_quaternion( Quat1);
    Quat_Product = quatmult( Quat0, Quat1);
    set(handles.Quat_Product_0,'String',Quat_Product(1,1));
    set(handles.Quat_Product_1,'String',Quat_Product(2,1));
    set(handles.Quat_Product_2,'String',Quat_Product(3,1));
    set(handles.Quat_Product_3,'String',Quat_Product(4,1));
    
    %Write Euler axis & angle
    [Euler_Axis,Euler_Angle]=RotMatToEulerAngles(RotMat);
    set(handles.Euler_AxisX,'String',Euler_Axis(1,1));
    set(handles.Euler_AxisY,'String',Euler_Axis(2,1));
    set(handles.Euler_AxisZ,'String',Euler_Axis(3,1));
    set(handles.Euler_Angle,'String',Euler_Angle);
    
    %Write rotation vector
    [Euler_Axis,Euler_Angle]=RotMatToEulerAxis_Angle(RotMat);
     Rot_Vec = Obt_RotVec(Euler_Axis,Euler_Angle);
    set(handles.x,'String',Rot_Vec(1));
    set(handles.y,'String',Rot_Vec(2));
    set(handles.z,'String',Rot_Vec(3));
    
    %Write r_mat 
    Set_rotation_matrix(RotMat);
    
    set(handles.RotMat_Pos1_1,'String',RotMat(1,1));
    set(handles.RotMat_Pos1_2,'String',RotMat(1,2));
    set(handles.RotMat_Pos1_3,'String',RotMat(1,3));

    set(handles.RotMat_Pos2_1,'String',RotMat(2,1));
    set(handles.RotMat_Pos2_2,'String',RotMat(2,2));
    set(handles.RotMat_Pos2_3,'String',RotMat(2,3));

    set(handles.RotMat_Pos3_1,'String',RotMat(3,1));
    set(handles.RotMat_Pos3_2,'String',RotMat(3,2));
    set(handles.RotMat_Pos3_3,'String',RotMat(3,3));
    
    last_cube = GetLastCube();
    handles.Cube = RedrawCube(RotMat,last_cube);
    Set_last_cube(handles.Cube);
    
    
    % --- Executes on button press in pushrotvec.
function pushrotvec_Callback(hObject, eventdata, handles)
% hObject    handle to pushrotvec (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
Set_last_cube(handles.Cube);

Rotation_Vector(1,1) = str2double(get(handles.x,'String'));
Rotation_Vector(2,1) = str2double(get(handles.y,'String'));
Rotation_Vector(3,1) = str2double(get(handles.z,'String'));

if(norm(Rotation_Vector)==1)
    Angle = 0;
else
    Angle = norm(Rotation_Vector);
end
Axis = Rotation_Vector/norm(Rotation_Vector);

RotMat=axisangle2matrix(Axis,Angle);

   %Write Euler Angles
    [Pitch,Roll,Yaw]=RotMatToEulerAngles(RotMat);
    set(handles.Pitch,'String',Pitch);
    set(handles.Roll,'String',Roll);
    set(handles.Yaw,'String',Yaw);    

 %Write quaternions
    if(isempty(GetLastQuaternion()))
        Quat0 = [0;0;0;0];
    else
        Quat0 = GetLastQuaternion();
    end
    set(handles.Quat0_0,'String', Quat0(1,1));
    set(handles.Quat0_1,'String', Quat0(2,1));
    set(handles.Quat0_2,'String', Quat0(3,1));
    set(handles.Quat0_3,'String', Quat0(4,1));

    Quat1 = EulerAngle_to_Quat(Pitch,Roll,Yaw);
    set(handles.Quat1_0,'String', Quat1(1,1));
    set(handles.Quat1_1,'String', Quat1(2,1));
    set(handles.Quat1_2,'String', Quat1(3,1));
    set(handles.Quat1_3,'String', Quat1(4,1));

    Set_last_quaternion(Quat1);
    
   Quat_Product = quatmult( Quat0, Quat1);
    set(handles.Quat_Product_0,'String',Quat_Product(1,1));
    set(handles.Quat_Product_1,'String',Quat_Product(2,1));
    set(handles.Quat_Product_2,'String',Quat_Product(3,1));
    set(handles.Quat_Product_3,'String',Quat_Product(4,1));
    
    
    %Write Euler axis & angle
    [Euler_Axis,Euler_Angle]=RotMatToEulerAngles(RotMat);
    set(handles.Euler_AxisX,'String',Euler_Axis(1,1));
    set(handles.Euler_AxisY,'String',Euler_Axis(2,1));
    set(handles.Euler_AxisZ,'String',Euler_Axis(3,1));
    set(handles.Euler_Angle,'String',Euler_Angle);
    
    
    %Write RotMat 
    Set_rotation_matrix(RotMat);
    
    set(handles.RotMat_Pos1_1,'String',RotMat(1,1));
    set(handles.RotMat_Pos1_2,'String',RotMat(1,2));
    set(handles.RotMat_Pos1_3,'String',RotMat(1,3));

    set(handles.RotMat_Pos2_1,'String',RotMat(2,1));
    set(handles.RotMat_Pos2_2,'String',RotMat(2,2));
    set(handles.RotMat_Pos2_3,'String',RotMat(2,3));

    set(handles.RotMat_Pos3_1,'String',RotMat(3,1));
    set(handles.RotMat_Pos3_2,'String',RotMat(3,2));
    set(handles.RotMat_Pos3_3,'String',RotMat(3,3));
    
    last_cube = GetLastCube();
    handles.Cube = RedrawCube(RotMat,last_cube);
    Set_last_cube(handles.Cube);
