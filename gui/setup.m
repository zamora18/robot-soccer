% Add folder to MATLAB's search path
% addpath('/Users/plusk01/Desktop/ecen490/robot-soccer/ros/src/matlab_gen/msggen');

% Get my IP Address (using the Java way)
address = java.net.InetAddress.getLocalHost;
IPaddress = char(address.getHostAddress);

setenv('ROS_MASTER_URI','http://ronald:11311')
setenv('ROS_IP',IPaddress) % my IP address

rosinit