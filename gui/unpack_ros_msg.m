function s = unpack_ros_msg( msg )
%UNPACK_ROBOTSTATE Unpack a custom ROS message into a struct
%   Expects a proper ROS msg (custom or otherwise, as long as the MATLAB
%   definition is present. See gen_msg.m for generating custom messages.)
%   and creates a struct with the same form.

    % Get a cell array of all the fields in this msg
    p = properties(msg);
    
    % From that cell array, create an empty struct
    s = cell2struct(cell(size(p)),p);
    
    % Populate each field in the struct, s
    for i = 1:length(p)
        if ischar(msg(i).(p{i}))
            s.(p{i}) = {msg.(p{i})};
        else
            s.(p{i}) = [msg.(p{i})];
        end
    end
end

