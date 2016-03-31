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

% Last Modified by GUIDE v2.5 30-Mar-2016 18:47:14

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

% Clear globals
global ball
ball = [];
global bot
bot = [];

global view_resp
view_resp = false;

% Choose default command line output for main
handles.output = hObject;

% Show the toolbar on the figure
set(gcf,'toolbar','figure');

% Setup Field Plot for Ally1 (and Opponent1)
handles.ally1 = main_gui_setup(...
        handles.ally1_field_ax,...
        handles.ally1_vel_ax,...
        handles.ally1_table_desired_position,...
        handles.ally1_table_velocity,...
        handles.ally1_table_position,...
        handles.ally1_table_error,...
        @field_ButtonDownCB,...
        1); % ally 1


% Setup ROS Sub/Pub
handles.ally1 = main_ros_setup( handles.ally1, 1,... % ally 1
        @robotStateCallback, @desiredPositionCallback, @velCmdsCallback,...
        @pidInfoCB, @ballStateCallback);
    
% % Setup Field Plot for Ally2 (and Opponent2)
% handles.ally2 = main_setup_gui(...
%         handles.ally2_field_ax,...
%         handles.ally2_vel_ax,...
%         handles.ally1_table_desired_position,...
%         handles.ally1_table_velocity,...
%         handles.ally1_table_position,...
%         handles.ally1_table_error,...
%         2); % ally 2
% 
% 
% % Setup ROS Sub/Pub
% handles.ally1 = main_ros_setup( handles.ally2, 2,... % ally 2
%         @robotStateCallback, @desiredPositionCallback, @velCmdsCallback,...
%         @pidInfoCB, @ballStateCallback);
    

% Set up ball stuff
set(handles.table_ball_vision,'Data', {0 0 0});
set(handles.table_ball_estimate,'Data', {0 0 0});


% Update handles structure
guidata(hObject, handles);

% UIWAIT makes main wait for user response (see UIRESUME)
% uiwait(handles.main);


% --- Outputs from this function are returned to the command line.
function varargout = main_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

% --- Executes when user attempts to close main.
function main_CloseRequestFcn(hObject, ~, handles)
% hObject    handle to main (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

clear handles

% The GUI is no longer waiting, just close it
delete(hObject);    


% --- Executes on button press in ally1_btn_clear_position.
function clearFieldCB(hObject, eventdata, handles)
% hObject    handle to ally1_btn_clear_position (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

if strcmp(hObject.Parent.Tag, 'ally1'),
    plot_ally_position = handles.ally1.plot_ally_position;
elseif strcmp(hObject.Parent.Tag, 'ally2'),
    plot_ally_position = handles.ally2.plot_ally_position;
end

set(plot_ally_position,'XData',0,'YData',0)


% --- Executes on button press in ally1_btn_set_desired_position.
function btn_setDesPosCB(hObject, eventdata, handles)
% hObject    handle to ally1_btn_set_desired_position (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
c = get(handles.ally1_table_desired_position,'Data');
desired = cell2mat(c);

msg = rosmessage(handles.pub.desired_position);
msg.X = desired(1);
msg.Y = desired(2);
msg.Theta = desired(3);
send(handles.pub.desired_position, msg);

% --- Executes on mouse press over axes background.
function field_ButtonDownCB(hObject, eventdata, handles)
% hObject    handle to ally1_field_ax (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

    handles = guidata(hObject);
    
    if strcmp(hObject.Parent.Tag, 'ally1'),
        chk_clickToDrive = handles.ally1_chk_clickToDrive;
    elseif strcmp(hObject.Parent.Tag, 'ally2'),
        chk_clickToDrive = handles.ally2_chk_point_move;
    end

    if get(chk_clickToDrive,'Value') == 1
        % Get the x,y point of the click
        point = eventdata.IntersectionPoint(1:2);
        
        % Create the first point of the line
        h1 = line('XData',point(1),'YData',point(2));
        set(h1,'Color','r');
        
        if strcmp(hObject.Parent.Tag, 'ally1'),
            ally = 1;
        elseif strcmp(hObject.Parent.Tag, 'ally2'),
            ally = 2;
        end
        
        hObject.Parent.Parent.WindowButtonMotionFcn = {@winBtnMotionCB,h1,point};
        hObject.Parent.Parent.WindowButtonUpFcn = {@winBtnUpCB,h1,ally};
    end

function winBtnMotionCB(hObject, ~, h1, p_init)

    % Get the x,y point that the mouse is hovering over.
    point = hObject.CurrentAxes.CurrentPoint(1,1:2);
    
    % Create a new line
    xdat = [p_init(1) point(1)];
    ydat = [p_init(2) point(2)];
    
    set(h1,'XData',xdat,'YData',ydat);
    
function winBtnUpCB(hObject, ~, h1, ally)
    global ally1_pos;
    global ally2_pos;

    handles = guidata(hObject);
    
    if ally == 1,
        if ~isempty(ally1_pos),
            current_theta = ally1_pos(2);
        else
            current_theta = 0;
        end
        
        pub_desired_position = handles.ally1.pub.desired_position;
        table_desired_position = handles.ally1.table_desired_position;
    elseif ally == 2,
        if ~isempty(ally2_pos),
            current_theta = ally2_pos(2);
        else
            current_theta = 0;
        end
        
        pub_desired_position = handles.ally2.pub.desired_position;
        table_desired_position = handles.ally2.table_desired_position;
    end

    % Clear the callbacks
    hObject.WindowButtonMotionFcn = '';
    hObject.WindowButtonUpFcn = '';

    % Get the line so we can calc angle
    xdat = get(h1, 'XData');
    ydat = get(h1, 'YData');

    if length(xdat) == 2

        theta = atan2(diff(ydat),diff(xdat));

        % Take care of the fact that atan2 returns [-pi, pi]
        if theta < 0
            theta = theta + 2*pi;
        end

        % Convert to degrees
        theta = theta*180/pi;
    else
        theta = current_theta;
    end

    delete(h1);
    
    % The set point is always the first place there was a click
    point = [xdat(1) ydat(1)];
        
    set(table_desired_position,'Data', {0 0 0});

    msg = rosmessage(pub_desired_position);
    msg.X = point(1);
    msg.Y = point(2);
    msg.Theta = theta;
    send(pub_desired_position, msg);

% --- Executes on button press in ally1_btn_stop_moving.
function btn_stopMovingCB(hObject, eventdata, handles)
% hObject    handle to ally1_btn_stop_moving (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

    if strcmp(hObject.Parent.Tag, 'ally1'),
        table_position = handles.ally1.table_position;
        pub_des_pos = handles.ally1.pub.desired_position;
    elseif strcmp(hObject.Parent.Tag, 'ally2'),
        table_position = handles.ally2.table_position;
        pub_des_pos = handles.ally2.pub.desired_position;
    end

    c = get(table_position,'Data');
    desired = cell2mat(c);

    msg = rosmessage(pub_des_pos);
    msg.X = desired(1);
    msg.Y = desired(2);
    msg.Theta = desired(3);
    send(pub_des_pos, msg);


% --- Executes on button press in ally1_btn_step_resp.
function btn_stepRespCB(hObject, eventdata, handles)
% hObject    handle to ally1_btn_step_resp (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global view_resp
global view_resp_start

view_resp_start = true;
view_resp = ~view_resp;

disp(view_resp);













%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ROS Callbacks
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function robotStateCallback(~, msg, handles, ally)

    if ~ishandle(handles.plot_ally_position) || ~ishandle(handles.plot_ally_position)
        return
    end
    
    global ally1_pos
    global ally2_pos
    global ally1_bot
    global ally2_bot
    
    % Save for when we are clicking to drive
    if ally == 1,
        ally1_pos = [msg.Xhat msg.Yhat msg.Thetahat];
        ally1_bot = [ally1_bot msg];
    elseif ally == 2,
        ally2_pos = [msg.Xhat msg.Yhat msg.Thetahat];
        ally2_bot = [ally2_bot msg];        
    end
    
    if ally == 1 || ally == 2,
        x = get(handles.plot_ally_position,'XData');
        y = get(handles.plot_ally_position,'YData');
        x = [x msg.Xhat];
        y = [y msg.Yhat];
        set(handles.plot_ally_position,'XData',x,'YData',y);
        set(handles.field_ax, 'ButtonDownFcn', {@field_ButtonDownCB,ally});
        set(handles.table_position,'Data', {msg.Xhat msg.Yhat msg.Thetahat});
    else
        set(handles.plot_opp_position,'XData',msg.Xhat,'YData',msg.Yhat);
    end

    
%     % Predicted (red X)
%     set(handles.plot_ball_estimate,'XData', msg.XhatFuture, 'YData', msg.YhatFuture);
%     set(handles.table_ball_estimate,'Data', {msg.XhatFuture msg.YhatFuture});
    
    % Estimated (black asterisk)
    if msg.Correction && (ally == 1 || ally == 2)
        set(handles.plot_bot_vision,'XData', msg.VisionX, 'YData', msg.VisionY);
    end
    
% function desiredPositionCallback(~, msg, handles)
    
function desiredPositionCallback(~, msg, handles, ally)
    if ~ishandle(handles.table_desired_position)
        return
    end

    set(handles.table_desired_position,'Data', {msg.X msg.Y msg.Theta});

function velCmdsCallback(~, msg, handles, ally)
    if ~ishandle(handles.plot_velocity) || ~ishandle(handles.table_velocity)
        return
    end

    vx = msg.Linear.X;
    vy = msg.Linear.Y;
    w  = msg.Angular.Z;

    set(handles.plot_velocity,'XData',0,'YData',0,'UData',vx,'VData',vy);

    set(handles.table_velocity,'Data', {vx vy w});
    
function pidInfoCB(~, msg, handles, ally)
    global view_resp
    global view_resp_start
    
    persistent step_resp_plot
    
    if ~ishandle(handles.table_error) %|| isempty(step_resp_plot) || ~ishandle(step_resp_plot(1,1))
        return
    end

    set(handles.table_error,'Data', {msg.Error.X msg.Error.Y msg.Error.Theta});
    
    % Select the plots to subplot (if you want theta, add it)
    labelYs = {'x-position (m)', 'y-position (m)', 'theta (deg)'};
%     labelYs = {'x-position (m)', 'y-position (m)'};

    % How many subplots should there be?
    N = length(labelYs);
    
    if view_resp
        
        desired = [msg.Desired.X msg.Desired.Y msg.Desired.Theta];
        actual = [msg.Actual.X msg.Actual.Y msg.Actual.Theta];
        
        if view_resp_start
            view_resp_start = false;
            
            % clear the figure
            figure(2);
            clf;
            
            % Initialize handles
            step_resp_plot = zeros(2,N);
            ax = zeros(1,N);
            
            % Setup the subplots
            for i = 1:N
                ax(i) = subplot(N,1,i);
                step_resp_plot(1,i) = plot(0,desired(i));
                hold on;
                step_resp_plot(2,i) = plot(0,actual(i));
                ylabel(labelYs(i));
                xlabel('samples (n)');
                if i == 1
                    title(['Ally ', num2str(ally), ' Step Response']);
                end
            end
            
            % Make the zoom linked in the x-direction
%             linkaxes(ax(:), 'x');
        else
            for i = 1:N
                % Update the YData vector for actual
                ydat = [get(step_resp_plot(2,i),'YData') actual(i)];
                t = (0:(length(ydat)-1));
                set(step_resp_plot(2,i),'XData',t,'YData',ydat);

                % Update the YData vector for desired
                ydat = [get(step_resp_plot(1,i),'YData') desired(i)];
                t = (0:(length(ydat)-1));
                set(step_resp_plot(1,i),'XData',t,'YData',ydat);
            end
        end
        
        
    end
    
function ballStateCallback(src, msg, handles, ally)
    global ally1_ball
    global ally2_ball
    
    % For grabbing ball data to analyze later
    if ally == 1,
        ally1_ball = [ally1_ball msg];
    elseif ally == 2,
        ally2_ball = [ally2_ball msg];
    end
    
    % Get global handles to access ball tables
    gHandles = guidata(handles.field_ax);

    % Make sure these things even exist
    if ~ishandle(gHandles.table_ball_estimate) ...
            ||  ~ishandle(gHandles.table_ball_vision)
        return
    end

    % Predicted (red X)
    set(handles.plot_ball_estimate,'XData', msg.XhatFuture, 'YData', msg.YhatFuture);
    set(gHandles.table_ball_estimate,'Data', {msg.XhatFuture msg.YhatFuture});
    
    % Estimated (green circle)
    set(handles.plot_ball_vision,'XData', msg.Xhat, 'YData', msg.Yhat);
    set(gHandles.table_ball_vision,'Data', {msg.Xhat msg.Yhat});
    
    % Plot vision measured?
    % You'd have to use the bool 'Correction' to know if you should plot it
    % or not, as these come in faster than the camera. basically, if you
    % just straight plot these measurements it will jump between its
    % position and 0
