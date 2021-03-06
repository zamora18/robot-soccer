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
    
     v_c = strategy_strongOffense(robot,opponent,ball,P,t);
%     v_c = strategy_switchRoles(robot,opponent,ball,P,t,t==0);
    
%     v_c(1:3) = skill_followBallOnLine(robot(:,1), ball, -P.field_length/3, P);
%     v_c(4:6) = skill_followBallOnSegment(robot(:,2),ball,-P.field_length/4,-P.field_width/3,P.field_width/3,ball,P);
%     v_c(4:6) = play_guardGoal(robot(:,2),ball,P);
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


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Helper Function %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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
% strategy: strong defense
%   - If ball is behind both bots and at least one opponent is on the right
%   - have bot1 screen while bot2 goes to guard goal.
%   - bot2 defend goal
%   - 
function v_c = strategy_strongDefense(robot, opponent, ball, P, t)

    % Break out x variables for comparison
    ball_x = ball(1);
    bot1_x = robot(1,1);
    bot2_x = robot(1,2);
    opp1_x = opponent(1,1);
    opp2_x = opponent(1,2);
    
    v1 = [0 0 0];
    
    v2 = play_guardGoal(robot(:,2), ball, P);
    
    v_c = [v1; v2];
end

%-----------------------------------------
% strategy: switch roles
%   - Robots switch between offense and defense roles if:
%   	- ball has not moved for a while
%       - bots have not moved for a while
%       - if ball gets behind the offense (still in front of defender)
%   - sub strategy: strong defense
function v_c = strategy_switchRoles(robot, opponent, ball, P, t, flag)

    % Declare and initialize persistent variables
    persistent bot_position_d1;
    persistent switch_roles;
    persistent aggressiveness;
    persistent time_at_home;
    persistent t_d1;
    if (flag)
        bot_position_d1 = robot;
        switch_roles    = false;
        aggressiveness  = 0.5;
        time_at_home    = 0;
        t_d1            = 0;
    end
    
    % At each time interval delta, compare robots locations to old data
    delta = 1; % second
    epsilon = 0.15; % meters
    if (~mod(t,delta))
        % calculate the distance between old and new positions
        dist1 = utility_distanceOf(robot(1:2,1), bot_position_d1(1:2,1));
        dist2 = utility_distanceOf(robot(1:2,2), bot_position_d1(1:2,2));
        
        % Check if either bot has changed by more than epislon
        if (dist1 < epsilon && dist2 < epsilon)
            % switch roles
            switch_roles = ~switch_roles;
        end
        
        % update old data
        bot_position_d1 = robot;
    end
    
    % decide on defender and attacker
    if (switch_roles)
        attacker = robot(:,2);
        defender = robot(:,1);
    else
        attacker = robot(:,1);
        defender = robot(:,2);
    end
        
    % will the ball ever not move while the robots are still moving?
    % Maybe if they have really good defense (screening)? It seems unlikely
    
    % If the ball gets behind the offense then switch roles
    back_of_attacker = (attacker(1) - P.robot_radius);
    if (ball(1) < back_of_attacker)
        switch_roles    = ~switch_roles;
        tmp             = defender;
        defender        = attacker;
        attacker        = tmp;
    end
    
    % execute substrategy defense
    switch_play_x = P.field_width/8;
    be_aggressive = (time_at_home/t > .5);
    offense = ~(ball(1) < switch_play_x) || be_aggressive;
    
    % update how long the ball has been in home
    if (ball(1) < 0)
        time_at_home = time_at_home + (t - t_d1);
    end
    
    % if the defender touches the ball, make the defender the attacker
    % if it makes sense
    
    if (offense)
        v_attacker = play_rushGoal(attacker, ball, P);
        if (be_aggressive)
            v_defender = play_rushGoal(defender, ball, P);
%             v_defender = play_screen(defender, opponent(:,1), ball, P);
        else
            v_defender = skill_followBallOnLine(defender, ball, P.field_width/12, P);
        end
    else
        v_attacker = play_rushGoal(attacker, ball, P);
        v_defender = play_guardGoal(defender, ball, P);
    end
    
    if (switch_roles)
        v1 = v_defender;
        v2 = v_attacker;
    else
        v1 = v_attacker;
        v2 = v_defender;
    end
    
    v_c = [v1; v2];
    
    % update time for time step calcs
    t_d1 = t;
end

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
      v = skill_goToPointFaceGoal(robot, P.goal, P);
  else
      v = skill_goToPointFaceGoal(robot, position, P);
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
  
  theta = atan2(ball(2)+P.goal(2), ball(1)+P.goal(1));
  x_pos = -P.field_length/2 + P.goal_box_length*cos(theta);
  y_pos = P.goal_box_length*sin(theta);
  
  % If the ball is really close to my position then i should push forward
  % to push the ball away from the goal.  This will allow us to get the
  % ball farther away from our goal rather than keeping it close where
  % they can try to score.
  %
%   if ball(1) < x_pos + 0.05,
%       if ball(2) < abs(y_pos)+ 0.05,
%           x_pos = -P.field_length/2 + cos(theta) * P.goal_width*2;% / 2;
%           y_pos = sin(theta) * P.goal_width*2; % / 2;
%       end
%   end

  vx = -P.control_k_vx*(robot(1) - x_pos);
  vy = -P.control_k_vy*(robot(2) - y_pos);
  omega = -P.control_k_phi*(robot(3) - theta);
  
  
  v = [vx; vy; omega];  
  
end

%-----------------------------------------
% play: screen
%   - get in between ball and specified opponent bot
%   - face the opponent
function v = play_screen(robot, opponent, ball, P)
    % Calculate point between ball and the opponent
    midpoint = utility_midpointOf(ball, opponent(1:2));
    
    % Hold the position there
    v = skill_goToPointFaceTarget(robot, midpoint, opponent, P);
    
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
% skill: go to point and face the goal
%   - Goes to the specified point (x,y) with the heading
%     always toward the opponent's goal
function v=skill_goToPointFaceGoal(robot, point, P)
    v = skill_goToPointFaceTarget(robot, point, P.goal, P);
end

%-----------------------------------------
% skill: go to point and face the target
%   - Goes to the specified point (x,y) with the heading
%     always toward the given target
function v=skill_goToPointFaceTarget(robot, point, target, P)

    % control x position to go to desired x point
    vx = -P.control_k_vx*(robot(1)-point(1));
    
    % control y position to go to desired y point
    vy = -P.control_k_vy*(robot(2)-point(2));

    % control angle to point toward goal
    theta_d = atan2(target(2)-robot(2), target(1)-robot(1));
    omega = -P.control_k_phi*(robot(3) - theta_d);
    
    vmax = utility_maximizeVelocity([vx vy], P);
    vx = vmax(1);
    vy = vmax(2);
    
    v = [vx; vy; omega];
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Utilities %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%------------------------------------------
% utility: saturate velocity
%   - saturate the commanded velocity 
% function v = utility_saturateVelocity(v,P)
%     if v(1) >  P.robot_max_vx,    v(1) =  P.robot_max_vx;    end
%     if v(1) < -P.robot_max_vx,    v(1) = -P.robot_max_vx;    end
%     if v(2) >  P.robot_max_vy,    v(2) =  P.robot_max_vy;    end
%     if v(2) < -P.robot_max_vy,    v(2) = -P.robot_max_vy;    end
%     if v(3) >  P.robot_max_omega, v(3) =  P.robot_max_omega; end
%     if v(3) < -P.robot_max_omega, v(3) = -P.robot_max_omega; end
% end
function v = utility_saturateVelocity(v,P)
    if abs(v(1)) > P.robot_max_vx
        v(1:2) = v(1:2) / abs(v(1)) * P.robot_max_vx;
    end
    if abs(v(2)) > P.robot_max_vy
        v(1:2) = v(1:2) / abs(v(2)) * P.robot_max_vy;
    end
    if abs(v(3)) > P.robot_max_omega
        v(3) = v(3) / abs(v(3)) * P.robot_max_omega;
    end
end

%------------------------------------------
% utility: distance of
%   - find the distance between two points
function d = utility_distanceOf(p1, p2)
    d = norm(p1 - p2);
end

%------------------------------------------
% utility: midpoint of
%   - find the distance between two points
function p = utility_midpointOf(p1, p2)
    p = [(p1(1)+p2(1))/2 (p1(2)+p2(2))/2];
end

%------------------------------------------
% utility: maximize velocity
%   - 
function v = utility_maximizeVelocity(v, P)
    % Avoid division by zero
    if v(1) == 0 && v(2) == 0, return; end
    
    % If normalized vx is faster than normalized vy
    if abs(v(1)) / P.robot_max_vx > abs(v(2)) / P.robot_max_vy
        v = v / abs(v(1)) * P.robot_max_vx;
    else
        v = v / abs(v(2)) * P.robot_max_vy;
    end
end