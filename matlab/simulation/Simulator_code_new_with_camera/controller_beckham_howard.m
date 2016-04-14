% main control code - assumes full state knowledge
%
%
% Modified: 
%   2/11/2014 - R. Beard
%   2/18/2014 - R. Beard
%   2/24/2014 - R. Beard
%   1/4/2016  - R. Beard
%

%% this first function catches simulink errors and displays the line number
function v_c=controller_beckham_howard(uu,P)
    try
        v_c=controller_away_(uu,P);
    catch e
        msgString = getReport(e);
        fprintf(2,'\n%s\n',msgString);
        rethrow(e);
    end
end
%% AI Controler
function v_c=controller_away_(uu,P)
    % process inputs to function
    %% robots - own team
    for i=1:P.num_robots,
        robot(:,i)   = uu(1+3*(i-1):3+3*(i-1));
    end
    NN = 3*P.num_robots;
    %% robots - opponent
    for i=1:P.num_robots,
        opponent(:,i)   = uu(1+3*(i-1)+NN:3+3*(i-1)+NN);
    end
    NN = NN + 3*P.num_robots;
    %% ball
    ball = [uu(1+NN); uu(2+NN)];
    NN = NN + 2;
    %% score: own team is score(1), opponent is score(2)
    score = [uu(1+NN); uu(2+NN)];
    NN = NN + 2;
    %% current time
    t      = uu(1+NN);
    %% Robot 1 Control Commands
    % robot #1 positions itself behind ball and rushes the goal.
    %v1 = play_rush_goal(robot(:,1), ball, P);
    %v1 = skill_follow_ball_on_line(robot(:,1), ball, -P.field_width/3, P);
    %% Robot 2 Control Commands
    % robot #2 stays on line, following the ball, facing the goal
    v = coach(score, opponent(:,1), opponent(:,2), robot(:,1), robot(:,2), ball, P);
    v1 = v(1:3);
    v2 = v(4:6);
    %v2 = skill_follow_ball_on_line(robot(:,2), ball, -2*P.field_width/3, P);
    %v2 = skill_defend_goal(robot(:,2), ball, P);
    %v2 = play_rush_goal(robot(:,2), ball, P);
    %v2 = skill_go_to_point(robot(:,2), [-.1;-.5], P);
    %% Send the Velocity Commands through a saturation function
    v1 = utility_saturate_velocity(v1,P);
    v2 = utility_saturate_velocity(v2,P);
    v_c = [v1; v2];
end

%-----------------------------------------
% play - rush goal
%   - go to position behind ball
%   - if ball is between robot and goal, go to goal
% NOTE:  This is a play because it is built on skills, and not control
% commands.  Skills are built on control commands.  A strategy would employ
% plays at a lower level.  For example, switching between offense and
% defense would be a strategy.
function v = play_rush_goal(robot, ball, P, opponent1, opponent2)
  % normal vector from ball to goal
  n = P.goal-ball;
  n = n/norm(n);
  % compute position 10cm behind ball, but aligned with goal.
  position = ball - 0.2*n;
  flag = 0;
  if (opponent1(1) - robot(1) < 0.1) && (opponent1(1) > robot(1)) && (abs(opponent1(2) - robot(2)) < 0.2),
      flag = 1;
  elseif (opponent2(1) - robot(1) < 0.1) && (opponent2(1) > robot(1)) && (abs(opponent2(2) - robot(2)) < 0.2),
      flag = 1;
  end
  if norm(position-robot(1:2))<.21,
      v = skill_go_to_point(robot, P.goal, P, flag, ball);
  else
      v = skill_go_to_point(robot, position, P, flag, ball);
  end

end

%-----------------------------------------
% skill - follow ball on line
%   follows the y-position of the ball, while maintaining x-position at
%   x_pos.  Angle always faces the goal.

function v=skill_follow_ball_on_line(robot, ball, x_pos, P)

    % control x position to stay on current line
    vx = -P.control_k_vx*(robot(1)-x_pos);
    
    % control y position to match the ball's y-position
    vy = -P.control_k_vy*(robot(2)-ball(2));

    % control angle to -pi/2
    theta_d = atan2(P.goal(2)-robot(2), P.goal(1)-robot(1));
    omega = -P.control_k_phi*(robot(3) - theta_d); 
    
    v = [vx; vy; omega];
end

%-----------------------------------------
% skill - go to point
%   follows the y-position of the ball, while maintaining x-position at
%   x_pos.  Angle always faces the goal.

function v=skill_go_to_point(robot, point, P, flag, ball)

    % control x position to stay on current line
    vx = -P.control_k_vx*(robot(1)-point(1));
    
    % control y position to match the ball's y-position
    vy = -P.control_k_vy*(robot(2)-point(2));

    % control angle to -pi/2
    if flag == 0,
        theta_d = atan2(P.goal(2)-robot(2), P.goal(1)-robot(1));
    else
        theta_d = atan2(P.goal(2)-robot(2), P.goal(1)-robot(1)*1.5);
    end
    omega = -P.control_k_phi*(robot(3) - theta_d);   

    v = [vx; vy; omega];
end

%% Function designed to simulate the goalie role for the AI
function v=skill_defend_goal(robot, ball, P)
    % compute angle between ball and goal
    theta_ball = atan2(P.goal(2) + ball(2), P.goal(1) + ball(1));
    x_pos = -P.field_length / 2  + cos(theta_ball) * P.goal_width / 1.7;
    y_pos = sin(theta_ball) * P.goal_width / 1.7;
    
    % If the ball is really close to my position then i should push forward
    % to push the ball away from the goal.  This will allow us to get the
    % ball farther away from our goal rather than keeping it close where
    % they can try to score.
    %
    if ball(1) < x_pos + 0.1,
        if ball(2) < abs(y_pos)+ 0.1,
            x_pos = -P.field_length / 2 + cos(theta_ball) * P.goal_width*2;% / 2;
            y_pos = sin(theta_ball) * P.goal_width*2;% / 2;
        end
    end

    vx = -P.control_k_vx*(robot(1)-x_pos);
    vy = -P.control_k_vy*(robot(2)-y_pos);
    omega = -P.control_k_phi*(robot(3)-theta_ball);
    
    v = [vx; vy; omega];
end

%
function v = coach(score, opponent1, opponent2, robot1, robot2, ball, P)

    if (robot1(1) < robot2(1)) && (abs(robot1(2) < abs(robot2(2))))
        if ball(1) < P.field_length
            v1 = skill_defend_goal(robot1, ball, P);    
        else
            v1 = skill_follow_ball_on_line(robot1, ball, 0, P);
        end
        v2 = play_rush_goal(robot2, ball, P, opponent1, opponent2);
    else
        v1 = play_rush_goal(robot1, ball, P, opponent1, opponent2);
        if ball(1) < P.field_length
            v2 = skill_defend_goal(robot2, ball, P);    
        else
            v2 = skill_follow_ball_on_line(robot2, ball, 0, P);
        end
    end
    v = [v1; v2];
%     if robot1 < robot2
%         then robot1 => defense
%     else
%         then robot2 => defense
%     end
%     if I am losing the game
%         then I should play a little more defensivly
%     elseif our opponents are really close to their goal
%         then I should go and force him to one side of the goal
%     elseif they are really close together and close to our goal
%         I should start my power play
%     else 
%         I should play the game normally and have one defender and 
%     end      
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


  