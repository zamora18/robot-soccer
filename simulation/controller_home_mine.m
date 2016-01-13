% main control code - assumes full state knowledge
%
%
% Modified: 
%   2/11/2014 - R. Beard
%   2/18/2014 - R. Beard
%   2/24/2014 - R. Beard
%   1/4/2016  - R. Beard
%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Main %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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
function v_c=controller_home_(uu,P)

    % process inputs to function
    % robots - own team
    for i=1:P.num_robots,
        robot(:,i)   = uu(1+3*(i-1):3+3*(i-1));
    end
    NN = 3*P.num_robots;
    % robots - opponent
    for i=1:P.num_robots,
        opponent(:,i)   = uu(1+3*(i-1)+NN:3+3*(i-1)+NN);
    end
    NN = NN + 3*P.num_robots;
    % ball
    ball = [uu(1+NN); uu(2+NN)];
    NN = NN + 2;
    % score: own team is score(1), opponent is score(2)
    score = [uu(1+NN); uu(2+NN)];
    NN = NN + 2;
    % current time
    t      = uu(1+NN);

    %------------
    % Choose the strategy to perform
    
    % TODO: Build strategy picker
    
    v_c = strategy_strongOffense(robot,opponent,ball,P,t);
    
%     v_c(1:3) = skill_followBallOnLine(robot(:,1), ball, -P.field_length/3, P);
%     v_c(4:6) = skill_followBallOnSegment(robot(:,2),ball,-P.field_length/4,-P.field_width/3,P.field_width/3,ball,P);
    %------------
    
    % break up v_c into bots again
    v = reshape(v_c, 3, []);

    % output velocity commands to robots
    v1 = utility_saturateVelocity(v(:,1),P);
    v2 = utility_saturateVelocity(v(:,2),P);
    v_c = [v1; v2];
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
    
    %set some configuration variables
    guard_at_x = -P.field_length/12; % After the ball and bot go past this point,
                    % the other bot switches into defense mode at that point

    % Predicates
    time_to_guard = (ball_x > guard_at_x) && (bot1_x > guard_at_x);
                    
    % Bot1 rushes goal
    v1 = play_rushGoal(robot(:,1), ball, P);
    
    % Bot2 depends on the time_to_guard predicate
    if (time_to_guard)
        v2 = skill_followBallOnLine(robot(:,2), ball, guard_at_x, P);
    else
        v2 = play_rushGoal(robot(:,2), ball, P);
    end

    v_c = [v1; v2];
end

%-----------------------------------------
% strategy: strong defense
%   - do something
%   - do something else

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Plays %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%
% Offensive 
%%%%%%%%%%%%%

%-----------------------------------------
% play: rush goal
%   - go to position behind ball
%   - if ball is between robot and goal, go to goal
function v = play_rushGoal(robot, ball, P)
  
  % normal vector from ball to goal
  n = P.goal-ball;
  n = n/norm(n);
  % compute position 10cm behind ball, but aligned with goal.
  position = ball - 0.2*n;
    
  if norm(position-robot(1:2))<.2025,
      v = skill_goToPoint(robot, P.goal, P);
  else
      v = skill_goToPoint(robot, position, P);
  end

end

%%%%%%%%%%%%%
% Defensive 
%%%%%%%%%%%%%

%-----------------------------------------
% play: guard goal
%   - stay in goalie box
%   - stay behind ball
%   - keep heading toward ball
function v = play_guardGoal(robot, ball, P)
  
%   % normal vector from ball to goal
%   n = P.goal-ball;
%   n = n/norm(n);
%   % compute position 10cm behind ball, but aligned with goal.
%   position = ball - 0.2*n;
  
  % Set bounds of the segment (i.e., the goalie box)
  y_min = -P.field_width/3;
  y_max =  P.field_width/3;
  
  % set x position to stay at
  x_pos = -P.field_length/3;
  
  v = skill_followBallOnSegment(robot,ball,x_pos,y_min,y_max,ball,P);
    
%   if norm(position-robot(1:2))<.2025,
%       v = skill_goToPoint(robot, P.goal, P);
%   else
%       v = skill_goToPoint(robot, position, P);
%   end

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Skills %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%-----------------------------------------
% skill: follow ball on line
%   - follows the y-position of the ball, while maintaining x-position at
%     x_pos.  Angle always faces the goal.
function v=skill_followBallOnLine(robot, ball, x_pos, P)
    y_min = -P.field_width/2;
    y_max =  P.field_width/2;

    v = skill_followBallOnSegment(robot,ball,x_pos,y_min,y_max,P.goal,P);
end

%-----------------------------------------
% skill: follow ball on segment
%   - follows the y-position of the ball, while maintaining x-position at
%     x_pos. The bot will stay inside the segment defined by y_min & y_max. 
%     Angle always faces the target.
function v=skill_followBallOnSegment(robot, ball, x_pos, y_min, y_max, target, P)

    % define predicates
    ball_above_segment  = (ball(2) > y_max);
    ball_below_segment  = (ball(2) < y_min);

    % control x position to stay on current line
    vx = -P.control_k_vx*(robot(1)-x_pos);
    
    if (ball_above_segment)
        % control y position to stay at the top of the segment
        vy = -P.control_k_vy*(robot(2)-y_max);
    elseif (ball_below_segment)
        % control y position to stay at the bottom of the segment
        vy = -P.control_k_vy*(robot(2)-y_min);
    else
        % control y position to match the ball's y-position
        vy = -P.control_k_vy*(robot(2)-ball(2));
    end
    
    % control angle to face target
    theta_d = atan2(target(2)-robot(2), target(1)-robot(1));
    omega = -P.control_k_phi*(robot(3) - theta_d); 
    
    v = [vx; vy; omega];
end

%-----------------------------------------
% skill: go to point
%   - Goes to the specified point (x,y) with the heading
%     always toward the opponent's goal
function v=skill_goToPoint(robot, point, P)

    % control x position to go to desired x point
    vx = -P.control_k_vx*(robot(1)-point(1));
    
    % control y position to go to desired y point
    vy = -P.control_k_vy*(robot(2)-point(2));

    % control angle to point toward goal
    theta_d = atan2(P.goal(2)-robot(2), P.goal(1)-robot(1));
    omega = -P.control_k_phi*(robot(3) - theta_d); 
    
    v = [vx; vy; omega];
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Utilities %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%------------------------------------------
% utility: saturate velocity
%   - saturate the commanded velocity 
%
function v = utility_saturateVelocity(v,P)
    if v(1) >  P.robot_max_vx,    v(1) =  P.robot_max_vx;    end
    if v(1) < -P.robot_max_vx,    v(1) = -P.robot_max_vx;    end
    if v(2) >  P.robot_max_vy,    v(2) =  P.robot_max_vy;    end
    if v(2) < -P.robot_max_vy,    v(2) = -P.robot_max_vy;    end
    if v(3) >  P.robot_max_omega, v(3) =  P.robot_max_omega; end
    if v(3) < -P.robot_max_omega, v(3) = -P.robot_max_omega; end
end


  