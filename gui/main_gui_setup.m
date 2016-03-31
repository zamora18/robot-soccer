function [ h ] = main_gui_setup( field_ax, vel_ax,...
                                 t_desired_pos, t_vel, t_pos,...
                                 t_PID_error,...
                                 field_ButtonDownCB,...
                                 ally)
%MAIN_GUI_SETUP Summary of this function goes here
%   Detailed explanation goes here

    % Setup field
    h = setup_field_plots( field_ax, ally, field_ButtonDownCB );

    % Setup velocity plot
    h = setup_velocity_plots( h, vel_ax );

    % Initialize tables
    set(t_desired_pos,'Data', {0 0 0});
    set(t_vel,'Data', {0 0 0});
    set(t_pos,'Data', {0 0 0});
    set(t_PID_error,'Data', {0 0 0});
    
    % Rename and put into struct
    h.field_ax = field_ax;
    h.vel_ax = vel_ax;
    h.table_desired_position = t_desired_pos;
    h.table_velocity = t_vel;
    h.table_position = t_pos;
    h.table_error = t_PID_error;

end

function h = setup_field_plots( field_ax, ally, field_ButtonDownCB )

    % Create the position plot by putting it in the middle of the field
    h.plot_ally_position = plot(field_ax, 0, 0); hold(field_ax,'on');
    
    % Create the opponent plot
    h.plot_opp_position = plot(field_ax, 0, 0, 'ks');

    % Create a black * for where the vision says the bot is. You should see
    % jumps as the blue line pushes ahead (the estimator).
    h.plot_bot_vision = plot(field_ax,0,0,'k*');

    % Create a red circle for where the vision says the ball actually is
    h.plot_ball_vision = plot(field_ax, 0, 0, 'ro');

    % Create a green x for where the predictor says the ball will be
    h.plot_ball_estimate = plot(field_ax,0,0,'gx');

    % Make the field look nice
    set(field_ax,'XLim',[-2 2],'YLim',[-1.6 1.6]);
    daspect(field_ax, [1 1 1]);
    xlabel(field_ax, 'width (meters)');
    ylabel(field_ax, 'height (meters)');
    set(field_ax, 'XGrid', 'on', 'YGrid', 'on');

    % Mouse down callback to handle 'Click to Drive'
    set(field_ax, 'ButtonDownFcn', {field_ButtonDownCB, ally});

end

function h = setup_velocity_plots( h, vel_ax )

    h.plot_velocity = quiver(vel_ax,0,0,0,0,0);
    set(vel_ax,'XLim',[-1.5 1.5],'YLim',[-1.5 1.5]);
    daspect(vel_ax, [1 1 1]);
    xlabel(vel_ax,'x (m/s)');
    ylabel(vel_ax,'y (m/s)');
    set(vel_ax, 'XGrid', 'on', 'YGrid', 'on');
    
end