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

% Last Modified by GUIDE v2.5 04-Jan-2019 22:47:39

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
    Set_init_vec(RotVec(3,xmouse,ymouse));
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
    
    Init_Vector = Get_init_vec();
    Axis = -cross(RotVec(3,xmouse,ymouse), Init_Vector);
    Angle = acosd((RotVec(3,xmouse,ymouse)'*Init_Vector)/(norm(RotVec(3,xmouse,ymouse))*norm(Init_Vector)))*0.2;
    
    Rot_Mat = axisangle2matrix(Axis, Angle);
    
    %Write quaternions
    if(isempty(GetLastQuaternion()))
        Quat0 = [0;0;0;0];
    else
        Quat0 = GetLastQuaternion();
    end
    set(handles.Quat0_0,'String',Quat0(1,1));
    set(handles.Quat0_1,'String',Quat0(2,1));
    set(handles.Quat0_2,'String',Quat0(3,1));
    set(handles.Quat0_3,'String',Quat0(4,1));

    Quat1 = TwoVec_To_Quat(Init_Vector, RotVec(3,xmouse,ymouse));
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

    %Write Euler Axis & Angle
    [Euler_Axis,Euler_Angle]=RotMatToEulerAxis_Angle(Rot_Mat);
    Euler_Axis=Euler_Axis';
    set(handles.Euler_AxisX,'String',Euler_Axis(1,1));
    set(handles.Euler_AxisY,'String',Euler_Axis(2,1));
    set(handles.Euler_AxisZ,'String',Euler_Axis(3,1));
    set(handles.Euler_Angle,'String',Euler_Angle);
   
    %Write Euler Angles
    [Roll,Pitch,Yaw]=RotMatToEulerAngles(Rot_Mat);
    set(handles.Pitch,'String',Pitch);
    set(handles.Roll,'String',Roll);
    set(handles.Yaw,'String',Yaw);

    %Write rotation vector %% SEE ORIGINAL, PARAMETER REPEAT
    [Euler_Axis,Euler_Angle]=RotMatToEulerAxis_Angle(Rot_Mat);
     Rotation_Vector = Obt_RotVec(Euler_Axis,Euler_Angle);
    set(handles.X,'String',Rotation_Vector(1));
    set(handles.Y,'String',Rotation_Vector(2));
    set(handles.Z,'String',Rotation_Vector(3));

    
    %Write r_mat 
    Set_rotation_matrix(Rot_Mat);

    set(handles.RotMat_Pos1_1,'String',Rot_Mat(1,1));
    set(handles.RotMat_Pos1_2,'String',Rot_Mat(1,2));
    set(handles.RotMat_Pos1_3,'String',Rot_Mat(1,3));

    set(handles.RotMat_Pos2_1,'String',Rot_Mat(2,1));
    set(handles.RotMat_Pos2_2,'String',Rot_Mat(2,2));
    set(handles.RotMat_Pos2_3,'String',Rot_Mat(2,3));

    set(handles.RotMat_Pos3_1,'String',Rot_Mat(3,1));
    set(handles.RotMat_Pos3_2,'String',Rot_Mat(3,2));
    set(handles.RotMat_Pos3_3,'String',Rot_Mat(3,3));
    
    handles.Cube = RedrawCube(Rot_Mat,handles.Cube);    
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

%% new stuff

function Set_init_vec(v)
global initial_vector;
initial_vector = v;

function Set_rotation_matrix(r_mat)
global rotation_mat;
rotation_mat = r_mat;

function Set_last_quaternion(l_q)
global last_quat;
last_quat = l_q;

function Set_last_cube(l_c)
global last_cube;
last_cube = l_c;

function vec = Get_init_vec();
global initial_vector;
vec = initial_vector;

function mat = Get_rotation_matrix();
global rotation_mat;
mat = rotation_mat;

function quat = GetLastQuaternion();
global last_quat;
quat = last_quat;

function cube = GetLastCube();
global last_cube;
cube = last_cube;

function Quat0_0_Callback(hObject, eventdata, handles)
% hObject    handle to Quat0_0 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Quat0_0 as text
%        str2double(get(hObject,'String')) returns contents of Quat0_0 as a double


% --- Executes during object creation, after setting all properties.
function Quat0_0_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Quat0_0 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Quat0_1_Callback(hObject, eventdata, handles)
% hObject    handle to Quat0_1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Quat0_1 as text
%        str2double(get(hObject,'String')) returns contents of Quat0_1 as a double


% --- Executes during object creation, after setting all properties.
function Quat0_1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Quat0_1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Quat0_2_Callback(hObject, eventdata, handles)
% hObject    handle to Quat0_2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Quat0_2 as text
%        str2double(get(hObject,'String')) returns contents of Quat0_2 as a double


% --- Executes during object creation, after setting all properties.
function Quat0_2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Quat0_2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Quat0_3_Callback(hObject, eventdata, handles)
% hObject    handle to Quat0_3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Quat0_3 as text
%        str2double(get(hObject,'String')) returns contents of Quat0_3 as a double


% --- Executes during object creation, after setting all properties.
function Quat0_3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Quat0_3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Quat1_0_Callback(hObject, eventdata, handles)
% hObject    handle to Quat1_0 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Quat1_0 as text
%        str2double(get(hObject,'String')) returns contents of Quat1_0 as a double


% --- Executes during object creation, after setting all properties.
function Quat1_0_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Quat1_0 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Quat1_1_Callback(hObject, eventdata, handles)
% hObject    handle to Quat1_1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Quat1_1 as text
%        str2double(get(hObject,'String')) returns contents of Quat1_1 as a double


% --- Executes during object creation, after setting all properties.
function Quat1_1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Quat1_1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Quat1_2_Callback(hObject, eventdata, handles)
% hObject    handle to Quat1_2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Quat1_2 as text
%        str2double(get(hObject,'String')) returns contents of Quat1_2 as a double


% --- Executes during object creation, after setting all properties.
function Quat1_2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Quat1_2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Quat1_3_Callback(hObject, eventdata, handles)
% hObject    handle to Quat1_3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Quat1_3 as text
%        str2double(get(hObject,'String')) returns contents of Quat1_3 as a double


% --- Executes during object creation, after setting all properties.
function Quat1_3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Quat1_3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Quat_Product_0_Callback(hObject, eventdata, handles)
% hObject    handle to Quat_Product_0 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hints: get(hObject,'String') returns contents of Quat_Product_0 as text
%        str2double(get(hObject,'String')) returns contents of Quat_Product_0 as a double

% --- Executes during object creation, after setting all properties.
function Quat_Product_0_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Quat_Product_0 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function Quat_Product_1_Callback(hObject, eventdata, handles)
% hObject    handle to Quat_Product_1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hints: get(hObject,'String') returns contents of Quat_Product_1 as text
%        str2double(get(hObject,'String')) returns contents of Quat_Product_1 as a double

% --- Executes during object creation, after setting all properties.
function Quat_Product_1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Quat_Product_1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function Quat_Product_2_Callback(hObject, eventdata, handles)
% hObject    handle to Quat_Product_2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hints: get(hObject,'String') returns contents of Quat_Product_2 as text
%        str2double(get(hObject,'String')) returns contents of Quat_Product_2 as a double

% --- Executes during object creation, after setting all properties.
function Quat_Product_2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Quat_Product_2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function Quat_Product_3_Callback(hObject, eventdata, handles)
% hObject    handle to Quat_Product_3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hints: get(hObject,'String') returns contents of Quat_Product_3 as text
%        str2double(get(hObject,'String')) returns contents of Quat_Product_3 as a double

% --- Executes during object creation, after setting all properties.
function Quat_Product_3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Quat_Product_3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function Euler_AxisX_Callback(hObject, eventdata, handles)
% hObject    handle to Euler_AxisX (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hints: get(hObject,'String') returns contents of Euler_AxisX as text
%        str2double(get(hObject,'String')) returns contents of Euler_AxisX as a double

% --- Executes during object creation, after setting all properties.
function Euler_AxisX_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Euler_AxisX (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function Euler_AxisY_Callback(hObject, eventdata, handles)
% hObject    handle to Euler_AxisY (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hints: get(hObject,'String') returns contents of Euler_AxisY as text
%        str2double(get(hObject,'String')) returns contents of Euler_AxisY as a double

% --- Executes during object creation, after setting all properties.
function Euler_AxisY_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Euler_AxisY (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function Euler_AxisZ_Callback(hObject, eventdata, handles)
% hObject    handle to Euler_AxisZ (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Euler_AxisZ as text
%        str2double(get(hObject,'String')) returns contents of Euler_AxisZ as a double

% --- Executes during object creation, after setting all properties.
function Euler_AxisZ_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Euler_AxisZ (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function Euler_Angle_Callback(hObject, eventdata, handles)
% hObject    handle to Euler_Angle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hints: get(hObject,'String') returns contents of Euler_Angle as text
%        str2double(get(hObject,'String')) returns contents of Euler_Angle as a double

% --- Executes during object creation, after setting all properties.
function Euler_Angle_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Euler_Angle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function Pitch_Callback(hObject, eventdata, handles)
% hObject    handle to Pitch (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hints: get(hObject,'String') returns contents of Pitch as text
%        str2double(get(hObject,'String')) returns contents of Pitch as a double

% --- Executes during object creation, after setting all properties.
function Pitch_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Pitch (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function Roll_Callback(hObject, eventdata, handles)
% hObject    handle to Roll (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hints: get(hObject,'String') returns contents of Roll as text
%        str2double(get(hObject,'String')) returns contents of Roll as a double

% --- Executes during object creation, after setting all properties.
function Roll_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Roll (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function Yaw_Callback(hObject, eventdata, handles)
% hObject    handle to Yaw (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hints: get(hObject,'String') returns contents of Yaw as text
%        str2double(get(hObject,'String')) returns contents of Yaw as a double

% --- Executes during object creation, after setting all properties.
function Yaw_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Yaw (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function X_Callback(hObject, eventdata, handles)
% hObject    handle to X (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hints: get(hObject,'String') returns contents of X as text
%        str2double(get(hObject,'String')) returns contents of X as a double

% --- Executes during object creation, after setting all properties.
function X_CreateFcn(hObject, eventdata, handles)
% hObject    handle to X (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function Y_Callback(hObject, eventdata, handles)
% hObject    handle to Y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hints: get(hObject,'String') returns contents of Y as text
%        str2double(get(hObject,'String')) returns contents of Y as a double

% --- Executes during object creation, after setting all properties.
function Y_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function Z_Callback(hObject, eventdata, handles)
% hObject    handle to Z (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hints: get(hObject,'String') returns contents of Z as text
%        str2double(get(hObject,'String')) returns contents of Z as a double

% --- Executes during object creation, after setting all properties.
function Z_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Z (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes on button press in seteulerangles.
function setEulerAngles_Callback(hObject, eventdata, handles)
% hObject    handle to seteulerangles (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
Set_last_cube(handles.Cube);

Pitch = str2double(get(handles.Pitch,'String'));
Roll = str2double(get(handles.Roll,'String'));
Yaw = str2double(get(handles.Yaw,'String'));

Rot_Mat=EulerAnglesToRotMat(Roll,Pitch,Yaw);
%---------------------------Do All Again------------------------
  
    if(isempty(GetLastQuaternion()))
        Quat0 = [0;0;0;0];
    else
        Quat0 = GetLastQuaternion();
    end
    set(handles.Quat0_0,'String',Quat0(1,1));
    set(handles.Quat0_1,'String',Quat0(2,1));
    set(handles.Quat0_2,'String',Quat0(3,1));
    set(handles.Quat0_3,'String',Quat0(4,1));

    Quat1 = EulerAngle_to_Quat(Roll,Pitch,Yaw);
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

    %Write Euler Axis & Angle
    [Euler_Axis,Euler_Angle]=RotMatToEulerAxis_Angle(Rot_Mat);
     Euler_Axis=Euler_Axis';
    set(handles.Euler_AxisX,'String',Euler_Axis(1,1));
    set(handles.Euler_AxisY,'String',Euler_Axis(2,1));
    set(handles.Euler_AxisZ,'String',Euler_Axis(3,1));
    set(handles.Euler_Angle,'String',Euler_Angle);
   
    %Write Euler Angles
    [Roll,Pitch,Yaw]=RotMatToEulerAngles(Rot_Mat);
    set(handles.Pitch,'String',Pitch);
    set(handles.Roll,'String',Roll);
    set(handles.Yaw,'String',Yaw);

    %Write rotation vector 
    [Euler_Axis,Euler_Angle]=RotMatToEulerAxis_Angle(Rot_Mat);
     Rotation_Vector = Obt_RotVec(Euler_Axis,Euler_Angle);
    set(handles.X,'String',Rotation_Vector(1));
    set(handles.Y,'String',Rotation_Vector(2));
    set(handles.Z,'String',Rotation_Vector(3));

    
    %Write r_mat 
    Set_rotation_matrix(Rot_Mat);

    set(handles.RotMat_Pos1_1,'String',Rot_Mat(1,1));
    set(handles.RotMat_Pos1_2,'String',Rot_Mat(1,2));
    set(handles.RotMat_Pos1_3,'String',Rot_Mat(1,3));

    set(handles.RotMat_Pos2_1,'String',Rot_Mat(2,1));
    set(handles.RotMat_Pos2_2,'String',Rot_Mat(2,2));
    set(handles.RotMat_Pos2_3,'String',Rot_Mat(2,3));

    set(handles.RotMat_Pos3_1,'String',Rot_Mat(3,1));
    set(handles.RotMat_Pos3_2,'String',Rot_Mat(3,2));
    set(handles.RotMat_Pos3_3,'String',Rot_Mat(3,3));
     
    last_cube = GetLastCube();
    handles.Cube = RedrawCube(Rot_Mat,last_cube);
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

set(handles.X,'String',0);
set(handles.Y,'String',0);
set(handles.Z,'String',0);

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


% --- Executes on button press in seteuleraxis.
function seteuleraxis_Callback(hObject, eventdata, handles)
% hObject    handle to seteuleraxis (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
Set_last_cube(handles.Cube);

Euler_Angle = str2double(get(handles.Euler_Angle,'String'));
Euler_AxisX = str2double(get(handles.Euler_AxisX,'String'));
Euler_AxisY = str2double(get(handles.Euler_AxisY,'String'));
Euler_AxisZ = str2double(get(handles.Euler_AxisZ,'String'));

Rot_Mat=axisangle2matrix([Euler_AxisX;Euler_AxisY;Euler_AxisZ],Euler_Angle);
%---------------------------Do All Again------------------------
   %Write Euler Angles
    [Roll,Pitch,Yaw]=RotMatToEulerAngles(Rot_Mat);
    set(handles.Pitch,'String',Pitch);
    set(handles.Roll,'String',Roll);
    set(handles.Yaw,'String',Yaw);    

    %---------------------------
    %Write quaternions
    if(isempty(GetLastQuaternion()))
        Quat0 = [0;0;0;0];
    else
        Quat0 = GetLastQuaternion();
    end
    set(handles.Quat0_0,'String',Quat0(1,1));
    set(handles.Quat0_1,'String',Quat0(2,1));
    set(handles.Quat0_2,'String',Quat0(3,1));
    set(handles.Quat0_3,'String',Quat0(4,1));

   Quat1 = EulerAngle_to_Quat(Roll,Pitch,Yaw);
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

    %Write Euler Axis & Angle
    [Euler_Axis,Euler_Angle]=RotMatToEulerAxis_Angle(Rot_Mat);
    Euler_Axis=Euler_Axis';
    set(handles.Euler_AxisX,'String',Euler_Axis(1,1));
    set(handles.Euler_AxisY,'String',Euler_Axis(2,1));
    set(handles.Euler_AxisZ,'String',Euler_Axis(3,1));
    set(handles.Euler_Angle,'String',Euler_Angle);
   

    %Write rotation vector %% SEE ORIGINAL, PARAMETER REPEAT
    [Euler_Axis,Euler_Angle]=RotMatToEulerAxis_Angle(Rot_Mat);
     Rotation_Vector = Obt_RotVec(Euler_Axis,Euler_Angle);
    set(handles.X,'String',Rotation_Vector(1));
    set(handles.Y,'String',Rotation_Vector(2));
    set(handles.Z,'String',Rotation_Vector(3));

    
    %Write r_mat 
    Set_rotation_matrix(Rot_Mat);

    set(handles.RotMat_Pos1_1,'String',Rot_Mat(1,1));
    set(handles.RotMat_Pos1_2,'String',Rot_Mat(1,2));
    set(handles.RotMat_Pos1_3,'String',Rot_Mat(1,3));

    set(handles.RotMat_Pos2_1,'String',Rot_Mat(2,1));
    set(handles.RotMat_Pos2_2,'String',Rot_Mat(2,2));
    set(handles.RotMat_Pos2_3,'String',Rot_Mat(2,3));

    set(handles.RotMat_Pos3_1,'String',Rot_Mat(3,1));
    set(handles.RotMat_Pos3_2,'String',Rot_Mat(3,2));
    set(handles.RotMat_Pos3_3,'String',Rot_Mat(3,3));
    %---------------------------
    
    
    
    last_cube = GetLastCube();
    handles.Cube = RedrawCube(Rot_Mat,last_cube);
    Set_last_cube(handles.Cube);

% --- Executes on button press in setquaternions.
function setquaternions_Callback(hObject, eventdata, handles)
% hObject    handle to setquaternions (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
Set_last_cube(handles.Cube);

Quat1(1,1) = str2double(get(handles.Quat1_0,'String'));
Quat1(2,1) = str2double(get(handles.Quat1_1,'String'));
Quat1(3,1) = str2double(get(handles.Quat1_2,'String'));
Quat1(4,1) = str2double(get(handles.Quat1_3,'String'));

[Pitch_AUX,Roll_AUX,Yaw_AUX]=Quat_To_EulerAngles(Quat1);
Rot_Mat=EulerAnglesToRotMat(Pitch_AUX,Roll_AUX,Yaw_AUX);
%---------------------------Do All Again------------------------
   

    %Write Euler Angles
    [Roll,Pitch,Yaw]=RotMatToEulerAngles(Rot_Mat);
    set(handles.Pitch,'String',Pitch);
    set(handles.Roll,'String',Roll);
    set(handles.Yaw,'String',Yaw);    

  
    %Write Euler Angles
    [Roll,Pitch,Yaw]=RotMatToEulerAngles(Rot_Mat);
    set(handles.Pitch,'String',Pitch);
    set(handles.Roll,'String',Roll);
    set(handles.Yaw,'String',Yaw);    

    %Write quaternions
    if(isempty(GetLastQuaternion()))
        Quat0 = [0;0;0;0];
    else
         Quat0 = GetLastQuaternion();
    end
     Quat1 = EulerAngle_to_Quat(Roll,Pitch,Yaw);
    Set_last_quaternion(Quat1);
    Quat_Product = quatmult(Quat0,Quat1);
    set(handles.Quat_Product_0,'String',Quat_Product(1,1));
    set(handles.Quat_Product_1,'String',Quat_Product(2,1));
    set(handles.Quat_Product_2,'String',Quat_Product(3,1));
    set(handles.Quat_Product_3,'String',Quat_Product(4,1))


    %Write Euler Axis & Angle
    [Euler_Axis,Euler_Angle]=RotMatToEulerAxis_Angle(Rot_Mat);
    Euler_Axis=Euler_Axis';
    set(handles.Euler_AxisX,'String',Euler_Axis(1,1));
    set(handles.Euler_AxisY,'String',Euler_Axis(2,1));
    set(handles.Euler_AxisZ,'String',Euler_Axis(3,1));
    set(handles.Euler_Angle,'String',Euler_Angle);
   

    %Write rotation vector %% SEE ORIGINAL, PARAMETER REPEAT
    [Euler_Axis,Euler_Angle]=RotMatToEulerAxis_Angle(Rot_Mat);
     Rotation_Vector = Obt_RotVec(Euler_Axis,Euler_Angle);
    set(handles.X,'String',Rotation_Vector(1));
    set(handles.Y,'String',Rotation_Vector(2));
    set(handles.Z,'String',Rotation_Vector(3));

    
    %Write r_mat 
    Set_rotation_matrix(Rot_Mat);

    set(handles.RotMat_Pos1_1,'String',Rot_Mat(1,1));
    set(handles.RotMat_Pos1_2,'String',Rot_Mat(1,2));
    set(handles.RotMat_Pos1_3,'String',Rot_Mat(1,3));

    set(handles.RotMat_Pos2_1,'String',Rot_Mat(2,1));
    set(handles.RotMat_Pos2_2,'String',Rot_Mat(2,2));
    set(handles.RotMat_Pos2_3,'String',Rot_Mat(2,3));

    set(handles.RotMat_Pos3_1,'String',Rot_Mat(3,1));
    set(handles.RotMat_Pos3_2,'String',Rot_Mat(3,2));
    set(handles.RotMat_Pos3_3,'String',Rot_Mat(3,3));
    
    last_cube = GetLastCube();
    handles.Cube = RedrawCube(Rot_Mat,last_cube);
    Set_last_cube(handles.Cube);


% --- Executes on button press in setrotvec.
function setrotvec_Callback(hObject, eventdata, handles)
% hObject    handle to setrotvec (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
Set_last_cube(handles.Cube);

Rotation_Vector(1,1) = str2double(get(handles.X,'String'));
Rotation_Vector(2,1) = str2double(get(handles.Y,'String'));
Rotation_Vector(3,1) = str2double(get(handles.Z,'String'));

Angle = norm(Rotation_Vector);

Axis = Rotation_Vector/norm(Rotation_Vector);

Rot_Mat=axisangle2matrix(Axis,Angle);
%---------------------------Do All Again------------------------
  
  %Write Euler Angles
    [Roll,Pitch,Yaw]=RotMatToEulerAngles(Rot_Mat);
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

    Quat1 = EulerAngle_to_Quat(Roll,Pitch,Yaw);
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
    [Euler_Axis,Euler_Angle]=RotMatToEulerAxis_Angle(Rot_Mat);
    Euler_Axis=Euler_Axis';
    set(handles.Euler_AxisX,'String',Euler_Axis(1,1));
    set(handles.Euler_AxisY,'String',Euler_Axis(2,1));
    set(handles.Euler_AxisZ,'String',Euler_Axis(3,1));
    set(handles.Euler_Angle,'String',Euler_Angle);
   

    %Write rotation vector %% SEE ORIGINAL, PARAMETER REPEAT
    [Euler_Axis,Euler_Angle]=RotMatToEulerAxis_Angle(Rot_Mat);
     Rotation_Vector = Obt_RotVec(Euler_Axis,Euler_Angle);
    set(handles.X,'String',Rotation_Vector(1));
    set(handles.Y,'String',Rotation_Vector(2));
    set(handles.Z,'String',Rotation_Vector(3));

    
    %Write r_mat 
    Set_rotation_matrix(Rot_Mat);

    set(handles.RotMat_Pos1_1,'String',Rot_Mat(1,1));
    set(handles.RotMat_Pos1_2,'String',Rot_Mat(1,2));
    set(handles.RotMat_Pos1_3,'String',Rot_Mat(1,3));

    set(handles.RotMat_Pos2_1,'String',Rot_Mat(2,1));
    set(handles.RotMat_Pos2_2,'String',Rot_Mat(2,2));
    set(handles.RotMat_Pos2_3,'String',Rot_Mat(2,3));

    set(handles.RotMat_Pos3_1,'String',Rot_Mat(3,1));
    set(handles.RotMat_Pos3_2,'String',Rot_Mat(3,2));
    set(handles.RotMat_Pos3_3,'String',Rot_Mat(3,3));
    
    last_cube = GetLastCube();
    handles.Cube = RedrawCube(Rot_Mat,last_cube);
    Set_last_cube(handles.Cube);

