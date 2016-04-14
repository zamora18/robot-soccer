function varargout = ball_vision_test(varargin)
% BALL_VISION_TEST MATLAB code for ball_vision_test.fig
%      BALL_VISION_TEST, by itself, creates a new BALL_VISION_TEST or raises the existing
%      singleton*.
%
%      H = BALL_VISION_TEST returns the handle to a new BALL_VISION_TEST or the handle to
%      the existing singleton*.
%
%      BALL_VISION_TEST('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in BALL_VISION_TEST.M with the given input arguments.
%
%      BALL_VISION_TEST('Property','Value',...) creates a new BALL_VISION_TEST or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before ball_vision_test_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to ball_vision_test_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help ball_vision_test

% Last Modified by GUIDE v2.5 29-Feb-2016 16:22:15

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @ball_vision_test_OpeningFcn, ...
                   'gui_OutputFcn',  @ball_vision_test_OutputFcn, ...
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


% --- Executes just before ball_vision_test is made visible.
function ball_vision_test_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to ball_vision_test (see VARARGIN)

% Choose default command line output for ball_vision_test
handles.output = hObject;

% init the sample count
global N
global count
N = 0;
count = 0;

% Setup Position Plot
handles.plot = scatter(handles.ax, [], []);
set(handles.ax,'XLim',[-2 2],'YLim',[-2 2]);

% Setup Tables
set(handles.table_R, 'Data', [{0 0 0}; {0 0 0}; {0 0 0}]);

% Setup ROS Subscribers
handles.sub.vision_position = rossubscriber('/vision_ball_position', 'geometry_msgs/Pose2D', {@visionPositionCallback,handles});

% Update handles structure
guidata(hObject, handles);

function visionPositionCallback(src, msg, handles)

    if ~ishandle(handles.plot) || ~ishandle(handles.table_R)
        return
    end
    
    global N
    global count
    global x
    global y
    global p
    
    if N == 0
        return
    end
    
    if count == N
        N = 0;
        count = 0;
        p = [x' y'];
        
        cov(p)
        
        disp('done');
        
        x = [];
        y = [];
        set(handles.plot,'XData',x);
        set(handles.plot,'YData',y);
        
        return;
    end
    
    x = get(handles.plot,'XData');
    y = get(handles.plot,'YData');
    x = [x msg.X];
    y = [y msg.Y];
    set(handles.plot,'XData',x,'YData',y);
    
    count = count + 1;
    
%     set(handles.table_R,'Data', {msg.X msg.Y msg.Theta});


% --- Outputs from this function are returned to the command line.
function varargout = ball_vision_test_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in btnGo.
function btnGo_Callback(hObject, eventdata, handles)
% hObject    handle to btnGo (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

str = get(handles.txtSampleCount, 'string');

global N
N = str2double(str);

function txtSampleCount_Callback(hObject, eventdata, handles)
% hObject    handle to txtSampleCount (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of txtSampleCount as text
%        str2double(get(hObject,'String')) returns contents of txtSampleCount as a double


% --- Executes during object creation, after setting all properties.
function txtSampleCount_CreateFcn(hObject, eventdata, handles)
% hObject    handle to txtSampleCount (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
