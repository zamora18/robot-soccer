function varargout = main(varargin)
% MAIN MATLAB code for main.fig
%      MAIN, by itself, creates a new MAIN or raises the existing
%      singleton*.
%
%      H = MAIN returns the handle to a new MAIN or the handle to
%      the existing singleton*.
%
%      MAIN('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in MAIN.M with the given input arguments.
%
%      MAIN('Property','Value',...) creates a new MAIN or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before main_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to main_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help main

% Last Modified by GUIDE v2.5 24-Feb-2016 01:19:09

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @main_OpeningFcn, ...
                   'gui_OutputFcn',  @main_OutputFcn, ...
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


% --- Executes just before main is made visible.
function main_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to main (see VARARGIN)

% Choose default command line output for main
handles.output = hObject;

% Setup Position Plot
handles.plot_position = plot(handles.fig_position,0,0);
hold(handles.fig_position,'on');
handles.plot_ball_vision = plot(handles.fig_position,0,0,'ro');
handles.plot_ball_estimate = plot(handles.fig_position,0,0,'gx');
set(handles.fig_position,'XLim',[-1.8 1.8],'YLim',[-1 1]);
daspect(handles.fig_position, [1 1 1]);
xlabel(handles.fig_position, 'width (meters)');
ylabel(handles.fig_position, 'height (meters)');
set(handles.fig_position, 'XGrid', 'on', 'YGrid', 'on');
set(handles.fig_position, 'ButtonDownFcn', @fig_position_ButtonDownFcn);

% Setup Velocity Plot
handles.plot_velocity = quiver(handles.fig_velocity,0,0,0,0,0);
set(handles.fig_velocity,'XLim',[-1.5 1.5],'YLim',[-1.5 1.5]);
daspect(handles.fig_velocity, [1 1 1]);
xlabel(handles.fig_velocity,'width (m/s)');
ylabel(handles.fig_velocity,'height (m/s)');
set(handles.fig_velocity, 'XGrid', 'on', 'YGrid', 'on');

% Setup Tables
set(handles.table_desired_position,'Data', {0 0 0});
set(handles.table_velocity,'Data', {0 0 0});
set(handles.table_position,'Data', {0 0 0});
set(handles.table_error,'Data', {0 0 0});
set(handles.table_ball_vision,'Data', {0 0 0});
set(handles.table_ball_estimate,'Data', {0 0 0});

% Setup ROS Subscribers
handles.sub.vision_robot_position = rossubscriber('/vision_robot_position', 'geometry_msgs/Pose2D', {@visionRobotPositionCallback,handles});
handles.sub.desired_position = rossubscriber('/desired_position', 'geometry_msgs/Pose2D', {@desiredPositionCallback,handles});
handles.sub.vel_cmds = rossubscriber('/vel_cmds', 'geometry_msgs/Twist', {@velCmdsCallback,handles});
handles.sub.error = rossubscriber('/error', 'geometry_msgs/Pose2D', {@errorCallback,handles});
handles.sub.vision_ball_position = rossubscriber('/vision_ball_position', 'geometry_msgs/Pose2D', {@visionBallPositionCallback,handles});
handles.sub.estimated_ball_position = rossubscriber('/estimated_ball_position', 'geometry_msgs/Pose2D', {@estimatedBallPositionCallback,handles});


% And Publishers
handles.pub.desired_position = rospublisher('/desired_position', 'geometry_msgs/Pose2D');

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes main wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = main_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

% --- Executes when user attempts to close figure1.
function figure1_CloseRequestFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

clear handles.sub
clear handles.pub

% The GUI is no longer waiting, just close it
delete(hObject);

function visionRobotPositionCallback(src, msg, handles)

    if ~ishandle(handles.plot_position) || ~ishandle(handles.plot_position)
        return
    end
    
    global pos
    pos = [msg.X msg.Y msg.Theta];

    x = get(handles.plot_position,'XData');
    y = get(handles.plot_position,'YData');
    x = [x msg.X];
    y = [y msg.Y];
    set(handles.plot_position,'XData',x,'YData',y);
    
    set(handles.fig_position, 'ButtonDownFcn', @fig_position_ButtonDownFcn);

    set(handles.table_position,'Data', {msg.X msg.Y msg.Theta});
    
function desiredPositionCallback(src, msg, handles)
    if ~ishandle(handles.table_desired_position)
        return
    end

    set(handles.table_desired_position,'Data', {msg.X msg.Y msg.Theta});

function velCmdsCallback(src, msg, handles)
    if ~ishandle(handles.plot_velocity) || ~ishandle(handles.table_velocity)
        return
    end

    vx = msg.Linear.X;
    vy = msg.Linear.Y;
    w  = msg.Angular.Z;

    set(handles.plot_velocity,'XData',0,'YData',0,'UData',vx,'VData',vy);

    set(handles.table_velocity,'Data', {vx vy w});
    
function errorCallback(src, msg, handles)
    if ~ishandle(handles.table_error)
        return
    end

    set(handles.table_error,'Data', {msg.X msg.Y msg.Theta});
    
    
function visionBallPositionCallback(src, msg, handles)
    if ~ishandle(handles.table_ball_vision)
        return
    end
    
    set(handles.plot_ball_vision,'XData', msg.X, 'YData', msg.Y);

    set(handles.table_ball_vision,'Data', {msg.X msg.Y});
    
    
function estimatedBallPositionCallback(src, msg, handles)
    if ~ishandle(handles.table_ball_estimate)
        return
    end

    set(handles.plot_ball_estimate,'XData', msg.X, 'YData', msg.Y);
    
    set(handles.table_ball_estimate,'Data', {msg.X msg.Y});
    


% --- Executes on button press in btn_clear_position.
function btn_clear_position_Callback(hObject, eventdata, handles)
% hObject    handle to btn_clear_position (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

set(handles.plot_position,'XData',0,'YData',0)


% --- Executes on button press in btn_set_desired_position.
function btn_set_desired_position_Callback(hObject, eventdata, handles)
% hObject    handle to btn_set_desired_position (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
c = get(handles.table_desired_position,'Data');
desired = cell2mat(c);

msg = rosmessage(handles.pub.desired_position);
msg.X = desired(1);
msg.Y = desired(2);
msg.Theta = desired(3);
send(handles.pub.desired_position, msg);


% --- Executes on button press in chk_point_move.
function chk_point_move_Callback(hObject, eventdata, handles)
% hObject    handle to chk_point_move (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of chk_point_move


% --- Executes on mouse press over axes background.
function fig_position_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to fig_position (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

    handles = guidata(hObject);
    global pos;
    
    if get(handles.chk_point_move,'Value') == 1
        point = eventdata.IntersectionPoint(1:2);
        
        theta = pos(2);


        set(handles.table_desired_position,'Data', {0 0 0});

        msg = rosmessage(handles.pub.desired_position);
        msg.X = point(1);
        msg.Y = point(2);
        msg.Theta = theta;
        send(handles.pub.desired_position, msg);
    end


% --- Executes on button press in chk_node_controller.
function chk_node_controller_Callback(hObject, eventdata, handles)
% hObject    handle to chk_node_controller (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    handles = guidata(hObject);
    if get(handles.chk_node_controller,'Value') == 1
        disp('Not implemented... Oops.');
    end


% --- Executes on button press in btn_update_status.
function btn_update_status_Callback(hObject, eventdata, handles)
% hObject    handle to btn_update_status (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    handles = guidata(hObject);
    
    get_battery = rossvcclient('/motion/main_battery');
    req = rosmessage(get_battery);
    resp = call(get_battery,req,'Timeout',3);
    
    set(handles.lbl_battery,'String',[num2str(resp) 'v']);
    
