% clear; close all; clc;
% load('bot_msg_data2.mat');

Tcamera=1/30;Tcontrol=1/100;
cam_latency = 130E-3; % s
update_type = 'SIMPLE';
Q = diag([(5E-2)^2 (5E-2)^2 (10*pi/180)^2]);
R = diag([ 0.001^2 0.001^2 (1*pi/180)^2 ]);
save('bot_msg_data7.mat');

% How many samples are there?
N = length(bot);

% Create a time vector
t = (0:N-1)*Tcontrol;

fprintf('Processing %f seconds of bot data.\r\n\r\n', N*Tcontrol);

% Initialize and unpackage
% x = [vision_ball(:).X];
% y = [vision_ball(:).Y];
x = nonzeros([bot(:).VisionX]);
y = nonzeros([bot(:).VisionY]);
theta = nonzeros([bot(:).VisionTheta]);
x_t = find([bot(:).VisionX]~=0)*Tcontrol;
y_t = find([bot(:).VisionY]~=0)*Tcontrol;
theta_t = find([bot(:).VisionTheta]~=0)*Tcontrol;

xhat = [bot(:).Xhat];
yhat = [bot(:).Yhat];
thetahat = [bot(:).Thetahat];
vx = [bot(:).Vx];
vy = [bot(:).Vy];
w = [bot(:).W];
xhat_future = [bot(:).XhatFuture];
yhat_future = [bot(:).YhatFuture];
thetahat_future = [bot(:).ThetahatFuture];
corrections = nonzeros([bot(:).Correction]);
corrections_t = find([bot(:).Correction]==1)*Tcontrol;

% Create a shifted corrections to be at mean of x and y
% The '-1' is because they are initially Booleans
corrections_x = (corrections-1) + mean(xhat);
corrections_y = (corrections-1) + mean(yhat);
corrections_theta = (corrections-1) + mean(thetahat);

figure(1); clf;
ax(1) = subplot(211);
plot(t,xhat,t,xhat_future,x_t,x);
legend('estimated','predicted','camera');
xlim([0 t(end)]);
title('x-position');
xlabel('time (s)');
ylabel('Position (m)');
hold on;
s = scatter(corrections_t,corrections_x, 'k');
set(s,'sizedata', .6);

% ax(2) = subplot(412);
% plot(t,vx);
% xlim([0 t(end)]);
% title('x-velocity');
% xlabel('time (s)');
% ylabel('Velocity (m/s)');

ax(3) = subplot(212);
plot(t,yhat,t,yhat_future,y_t,y);
legend('estimated','predicted','camera');
xlim([0 t(end)]);
title('y-position');
xlabel('time (s)');
ylabel('Position (m)');
hold on;
s = scatter(corrections_t,corrections_y, 'k');
set(s,'sizedata', .6);

% ax(4) = subplot(414);
% plot(t,vy);
% xlim([0 t(end)]);
% title('y-velocity');
% xlabel('time (s)');
% ylabel('Velocity (m/s)');

linkaxes(ax(:), 'x');

% Plot the initial position