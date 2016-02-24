% main control code 
%
%
% Modified: 
%   2/11/2014 - R. Beard
%   2/18/2014 - R. Beard
%   2/24/2014 - R. Beard
%   1/4/2016  - R. Beard
%   2/11/2016 - R. Beard - added Kalman filter
%

% this first function catches simulink errors and displays the line number
function v_c=controller_home(uu,P)
    try
        v_c=controller_home_(uu,P);
    catch e
        msgString = getReport(e);
        fprintf(2,'\n%s\n',msgString);
        rethrow(e);
    end
end

% main control function
function out=controller_home_(uu,P)
    [robot, opponent, ball, t] = utility_process_input(uu,P);
    
    persistent v_command
    if t==0,
        v_command = [zeros(3,1), zeros(3,1)];
    end
    
    robot    = utility_kalman_filter_robot(robot,v_command,t,P);
    opponent = utility_kalman_filter_opponent(opponent,t,P);
    ball     = utility_kalman_filter_ball(ball,t,P);

    %------------
    % Choose the strategy to perform    
    
     v_c = strategy_strongOffense(robot, opponent, ball, P, t);
%     v_c = strategy_switchRoles(robot,opponent,ball,P,t,t==0);
    
%     v_c(1:3) = skill_followBallOnLine(robot(:,1), ball, -P.field_length/3, P);
%     v_c(4:6) = skill_followBallOnSegment(robot(:,2),ball,-P.field_length/4,-P.field_width/3,P.field_width/3,ball,P);
%     v_c(4:6) = play_guardGoal(robot(:,2),ball,P);
    %------------
    
%     % robot #1 positions itself behind ball and rushes the goal.
%     v1 = play_rush_goal(robot(1), ball, P);
%  
%     % robot #2 stays on line, following the ball, facing the goal
%     v2 = skill_follow_ball_on_line(robot(2), ball, -2*P.field_width/3, P);


    %converting to v1 and v2 for compatibility with new code.
    v1 = v_c(1);
    v2 = v_C(2);
    
    % output velocity commands to robots
    v1 = utility_saturate_velocity(v1,P);
    v2 = utility_saturate_velocity(v2,P);
    v_command = [v1, v2];
    %out = [v1; v2];    
%    out = [v1; v2; ball.pos; reshape(ball.S(1:2,1:2),4,1)];
%    out = [v1; v2; opponent(1).pos; reshape(opponent(1).S(1:2,1:2),4,1)];
    out = [v1; v2; robot(1).pos; reshape(robot(1).S(1:2,1:2),4,1)];
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Strategies %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%-----------------------------------------
% strategy: strong offsense
%   - both bots rush ball to goal
%   - Once ball and bot1 are on the right half of the field,
%     bot2 follows ball on line at half of the pitch
% TODO: Perhaps don't assign roles to bot1/bot2 -- just act
%       based on whoever gets there first. Also, maybe make
%       the defensive bot move proportional to how close the ball/other bot
%       are to the goal.
function v_c = strategy_strongOffense(robot, opponent, ball, P, t)
    % Break out variables into something that makes sense
    ball_x = ball(1);
    bot1_x = robot(1,1);
    
    % set some configuration variables
    hard_line_of_defense = -P.field_length*3/8; %Both robots defend at goal.

%     v1 = skill_followBallOnLine(robot(:,1), ball, hard_line_of_defense, P);
%     v2 = play_guardGoal(robot(:,2), ball, P);
    
    line_A = -(P.field_length/6)*2;
    line_B = -(P.field_length/6);
    line_C = 0;
    line_D = (P.field_length/6);
    line_E = (P.field_length/6)*2;
    
    
    section = compute_section(ball(1), P);
    switch section
        case 1,
            v1 = skill_followBallOnLine(robot(:,1), ball*-1, line_A, P);
            v2 = play_guardGoal(robot(:,2), ball, P);
        case 2,
            closest_player_to_goal = get_closest_robot(robot, P.goal(1), P.goal(2));
            if (closest_player_to_goal == 1)
                v1 = play_guardGoal(robot(:,1), ball, P);
                v2 = play_rushGoal(robot(:,2), ball, P);
            else
                v1 = play_rushGoal(robot(:,1), ball, P);
                v2 = play_guardGoal(robot(:,2), ball, P); 
            end
            
        case 3,
            v1 = play_rushGoal(robot(:,1), ball, P);
            v2 = play_rushGoal(robot(:,2), ball, P);
%             v2 = play_guardGoal(robot(:,2), ball, P);
        case 4, 
            v1 = play_rushGoal(robot(:,1), ball, P);
            v2 = play_rushGoal(robot(:,2), ball, P);
        case 5,
            closest_player_to_ball = get_closest_robot(robot, ball(1), ball(2));
            if (closest_player_to_ball == 1)
                v1 = play_rushGoal(robot(:,1), ball, P);
                v2 = skill_followBallOnLine(robot(:,2), ball, line_C, P);
            else
                v1 = skill_followBallOnLine(robot(:,1), ball, line_C, P);
                v2 = play_rushGoal(robot(:,2), ball, P);
            end
%             v1 = play_rushGoal(robot(:,1), ball, P);
%             v2 = skill_followBallOnLine(robot(:,2), ball, line_C, P);
        case 6,
            closest_player_to_ball = get_closest_robot(robot, ball(1), ball(2));
            if (closest_player_to_ball == 1)
                v1 = play_rushGoal(robot(:,1), ball, P);
                v2 = skill_followBallOnLine(robot(:,2), ball, line_D, P);
            else
                v1 = skill_followBallOnLine(robot(:,1), ball, line_D, P);
                v2 = play_rushGoal(robot(:,2), ball, P);
            end
        otherwise
            v1 = play_rushGoal(robot(:,1), ball, P);
            v2 = play_guardGoal(robot(:,2), ball, P);
    end


%     % Predicates
% %     time_to_guard = (ball_x > guard_at_x) && (bot1_x > guard_at_x);
% %     time_to_guard = (ball_x < guard_at_x);
%     time_to_guard = false;
%     
%     % Bot1 depends on the time_to_guard predicate
%     if (time_to_guard)
%         v1 = skill_followBallOnLine(robot(:,1), ball, guard_at_x, P);
%     else
%         v1 = play_rushGoal(robot(:,1), ball, P);
%     end
% 
%     % Always goalie
%     v2 = play_guardGoal(robot(:,2), ball, P);
    
v_c = [v1; v2];
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Helper Functions %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function ball_section = compute_section(x_val, P)
% Field is divided into 6 sections.
% compute_section takes the x-position as input, and returns a number 1-6

x_pos = x_val;
if (x_pos <= 0) %Ball is in our half.
    if (x_pos < -(P.field_length/6)*2)
        section = 1;
    elseif (x_pos < -(P.field_length/6)*1) 
        section = 2;
    else
        section = 3;   
    end
    
else %Ball is in their half.
    if (x_pos < (P.field_length/6)*1)
        section = 4;
    elseif (x_pos < (P.field_length/6)*2)
        section = 5;
    else
        section = 6;
    end

end

ball_section = section;
end

function rob = get_closest_robot(robot, target_x, target_y)
% Gets the closest robot to a specified target
% Will return a 1 or 2 corresponding to robot_1 or robot_2

x_dist_rob_1 = robot(1,1) - target_x;
y_dist_rob_1 = robot(2,1) - target_y;
x_dist_rob_2 = robot(1,2) - target_x;
y_dist_rob_2 = robot(2,2) - target_y;

dist_rob_1 = sqrt(x_dist_rob_1^2 + y_dist_rob_1^2);
dist_rob_2 = sqrt(x_dist_rob_2^2 + y_dist_rob_2^2);


if (dist_rob_1 < dist_rob_2)
    rob = 1;
else
    rob = 2;
end

end





%-----------------------------------------
% play - rush goal
%   - go to position behind ball
%   - if ball is between robot and goal, go to goal
% NOTE:  This is a play because it is built on skills, and not control
% commands.  Skills are built on control commands.  A strategy would employ
% plays at a lower level.  For example, switching between offense and
% defense would be a strategy.
function v = play_rush_goal(robot, ball, P)
  
  % normal vector from ball to goal
  n = P.goal-ball.pos;
  n = n/norm(n);
  % compute position 10cm behind ball, but aligned with goal.
  position = ball.pos - 0.2*n;
    
  if norm(position-robot.pos)<.21,
      v = skill_go_to_point(robot, P.goal, P);
  else
      v = skill_go_to_point(robot, position, P);
  end

end

%-----------------------------------------
% skill - follow ball on line
%   follows the y-position of the ball, while maintaining x-position at
%   x_pos.  Angle always faces the goal.

function v=skill_follow_ball_on_line(robot, ball, x_pos, P)

    % control x position to stay on current line
    vx = -P.control_k_vx*(robot.pos(1)-x_pos);
    
    % control y position to match the ball's y-position
    vy = -P.control_k_vy*(robot.pos(2)-ball.pos(2));

    % control angle to -pi/2
    theta_d = atan2(P.goal(2)-robot.pos(2), P.goal(1)-robot.pos(1));
    omega = -P.control_k_phi*(robot.angle - theta_d); 
    
    v = [vx; vy; omega];
end

%-----------------------------------------
% skill - go to point
%   follows the y-position of the ball, while maintaining x-position at
%   x_pos.  Angle always faces the goal.

function v=skill_go_to_point(robot, point, P)

    % control x position to stay on current line
    vx = -P.control_k_vx*(robot.pos(1)-point(1));
    
    % control y position to match the ball's y-position
    vy = -P.control_k_vy*(robot.pos(2)-point(2));

    % control angle to -pi/2
    theta_d = atan2(P.goal(2)-robot.pos(2), P.goal(1)-robot.pos(1));
    omega = -P.control_k_phi*(robot.angle - theta_d); 
    
    v = [vx; vy; omega];
end

%------------------------------------------
% utility - process input and create robot, opponent, ball structures
%
function [robot, opponent, ball, t] = utility_process_input(uu,P)
    % robots - own team
    for i=1:P.num_robots,
        robot(i).pos_measurement   = [uu(1+3*(i-1));uu(2+3*(i-1))];
        robot(i).angle_measurement = uu(3+3*(i-1));
    end
    NN = 3*P.num_robots;
    % robots - opponent
    for i=1:P.num_robots,
        opponent(i).pos_measurement   = [uu(1+3*(i-1)+NN); uu(2+3*(i-1)+NN)];
        opponent(i).angle_measurement = uu(3+3*(i-1)+NN);
    end
    NN = NN + 3*P.num_robots;
    % ball
    ball.pos_measurement = [uu(1+NN);uu(2+NN)];
    NN = NN + 2;
    % score: own team is score(1), opponent is score(2)
    score = [uu(1+NN); uu(2+NN)];
    NN = NN + 2;
    % current time
    t      = uu(1+NN);
end


%------------------------------------------
% utility - saturate_velocity
% 	saturate the commanded velocity 
%
function v = utility_saturate_velocity(v,P)
    if v(1) >  P.robot_max_vx,    v(1) =  P.robot_max_vx;    end
    if v(1) < -P.robot_max_vx,    v(1) = -P.robot_max_vx;    end
    if v(2) >  P.robot_max_vy,    v(2) =  P.robot_max_vy;    end
    if v(2) < -P.robot_max_vy,    v(2) = -P.robot_max_vy;    end
    if v(3) >  P.robot_max_omega, v(3) =  P.robot_max_omega; end
    if v(3) < -P.robot_max_omega, v(3) = -P.robot_max_omega; end
end

%------------------------------------------
% utility - kalman filter for own team
%
function robot = utility_kalman_filter_robot(robot,v_command,t,P)
    persistent xhat
    persistent S
    for n=1:P.num_robots,    
        if t==0,  % initialize filter
            xhat(:,n) = [...
                0;... % initial guess at x-position of ownteam i
                0;... % initial guess at y-position of ownteam i
                0;... % initial guess at angle of ownteam i
                ];
        S(:,:,n) = diag([...
            P.field_width/2;... % initial variance of x-position of ownteam i
            P.field_width/2;... % initial variance of y-position of ownteam i 
            (5*pi/180)^2;... % initial variance of angle of ownteam i 
            ]);
        end
    
        % prediction step between measurements
        N = 10;
        for i=1:N,
            f = v_command(:,n);
            xhat(:,n) = xhat(:,n) + (P.control_sample_rate/N)*f;
            S(:,:,n) = S(:,:,n) + (P.control_sample_rate/N)*(P.Q_ownteam);
        end
 
        % correction step at measurement
        y_pred = xhat(:,n);  % predicted measurement
        L = S(:,:,n)/(P.R_ownteam+S(:,:,n));
        S(:,:,n) = (eye(3)-L)*S(:,:,n);
        xhat(:,n) = xhat(:,n) + L*( [robot(n).pos_measurement;robot(n).angle_measurement]-y_pred);
    
        % output current estimate of state
        robot(n).pos   = xhat(1:2,n);
        robot(n).angle = xhat(3,n);
        robot(n).S   = S(:,:,n);
    end    
end

%------------------------------------------
% utility - kalman filter for opponent team
%
function opponent = utility_kalman_filter_opponent(opponent,t,P)
    persistent xhat
    persistent S
    for n=1:P.num_robots,    
        if t==0,  % initialize filter
            xhat(:,n) = [...
                0;... % initial guess at x-position of opponent i
                0;... % initial guess at y-position of opponent i
                0;... % initial guess at angle of opponent i
                0;... % initial guess at x-velocity of opponent i
                0;... % initial guess at y-velocity of opponent i
                0;... % initial guess at angular velocity of opponent i
                0;... % initial guess at x-acceleration of opponent i
                0;... % initial guess at y-acceleration of opponent i
                0;... % initial guess at angular acceleration of opponent i
                ];
        S(:,:,n) = diag([...
            P.field_width/2;... % initial variance of x-position of opponent i
            P.field_width/2;... % initial variance of y-position of opponent i 
            (5*pi/180)^2;... % initial variance of angle of opponent i 
            .01;... % initial variance of x-velocity of opponent i 
            .01;... % initial variance of y-velocity of opponent i
            .01;... % initial variance of angular velocity of opponent i
            .001;... % initial variance of x-acceleration of opponent i 
            .001;... % initial variance of y-acceleration of opponent i
            .001;... % initial variance of angular acceleration of opponent i
            ]);
        end
    
        % prediction step between measurements
        N = 10; % 10 differential equation steps
        for i=1:N,
            f = [xhat(4:6,n); xhat(7:9,n); 0; 0; 0];
            xhat(:,n) = xhat(:,n) + (P.control_sample_rate/N)*f;
            A = [...
                zeros(3,3), eye(3), zeros(3,3);...
                zeros(3,3), zeros(3,3), eye(3);...
                zeros(3,3), zeros(3,3), zeros(3,3);...
                ];
            S(:,:,n) = S(:,:,n) + (P.control_sample_rate/N)*(A*S(:,:,n)+S(:,:,n)*A'+P.Q_opponent);
        end
 
        % correction step at measurement
        H = [eye(3), zeros(3,3), zeros(3,3)]; % H corresponds to C in his notes
        y_pred = H*xhat(:,n);  % predicted measurement
        L = S(:,:,n)*H'/(P.R_opponent+H*S(:,:,n)*H');
        S(:,:,n) = (eye(9)-L*H)*S(:,:,n);
        xhat(:,n) = xhat(:,n) + L*( [opponent(n).pos_measurement;opponent(n).angle_measurement]-y_pred);
    
        % output current estimate of state
        opponent(n).pos   = xhat(1:2,n);
        opponent(n).angle = xhat(3,n);
        opponent(n).vel   = xhat(4:5,n);
        opponent(n).angular_velocity = xhat(6,n);
        opponent(n).S   = S(:,:,n);
    end
end

%------------------------------------------
% utility - kalman filter for ball
%
function ball = utility_kalman_filter_ball(ball,t,P)
    persistent xhat
    persistent S
    
    if t==0,  % initialize filter
        xhat = [...
            0;... % initial guess at x-position of ball
            0;... % initial guess at y-position of ball
            0;... % initial guess at x-velocity of ball
            0;... % initial guess at y-velocity of ball
            0;... % initial guess at x-acceleration of ball
            0;... % initial guess at y-acceleration of ball
            0;... % initial guess at x-jerk of ball
            0;... % initial guess at y-jerk of ball
            ];
        S = diag([...
            P.field_width/2;... % initial variance of x-position of ball
            P.field_width/2;... % initial variance of y-position of ball 
            .01;... % initial variance of x-velocity of ball 
            .01;... % initial variance of y-velocity of ball
            .001;... % initial variance of x-acceleration of ball 
            .001;... % initial variance of y-acceleration of ball
            .0001;... % initial variance of x-jerk of ball 
            .0001;... % initial variance of y-jerk of ball
            ]);
    end
    
    % prediction step between measurements
    N = 10;
    for i=1:N,
        f = [xhat(3:4); xhat(5:6); xhat(7:8); 0; 0];
        xhat = xhat + (P.control_sample_rate/N)*f;
        A = [...
            zeros(2,2), eye(2), zeros(2,2), zeros(2,2);...
            zeros(2,2), zeros(2,2), eye(2), zeros(2,2);...
            zeros(2,2), zeros(2,2), zeros(2,2), eye(2);...
            zeros(2,2), zeros(2,2), zeros(2,2), zeros(2,2);...
            ];
        S = S + (P.control_sample_rate/N)*(A*S+S*A'+P.Q_ball);
    end
 
    % correction step at measurement
    H = [eye(2), zeros(2,2), zeros(2,2), zeros(2,2)];
    y_pred = H*xhat;  % predicted measurement
    L = S*H'/(P.R_ball+H*S*H');
    S = (eye(8)-L*H)*S;
    xhat = xhat + L*(ball.pos_measurement-y_pred);
    
    % output current estimate of state
    ball.pos   = xhat(1:2);
    ball.vel   = xhat(3:4);
    ball.accel = xhat(5:6);
    ball.jerk  = xhat(7:8);
    ball.S   = S;
end


  