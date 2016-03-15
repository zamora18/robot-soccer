% clear; close all; clc;
% load('ball_msg_data4.mat');

% Tcamera=1/30;
% Tcontrol=1/100;
% tau=0.075;
% alpha=0.75;
% save('ball_msg_data9.mat');

% How many samples are there?
N = length(ball);
% M = length(vision_ball);

% Create a time vector
t = (0:N-1)*Tcontrol;
% t_camera = (0:M-1)*Tcamera;

fprintf('Processing %f seconds of ball data.\r\n\r\n', N*Tcontrol);

% Initialize and unpackage
% x = [vision_ball(:).X];
% y = [vision_ball(:).Y];
x = nonzeros([ball(:).VisionX]);
y = nonzeros([ball(:).VisionY]);
x_t = find([ball(:).VisionX]~=0)*Tcontrol;
y_t = find([ball(:).VisionY]~=0)*Tcontrol;

xhat = [ball(:).Xhat];
yhat = [ball(:).Yhat];
vx = [ball(:).Vx];
vy = [ball(:).Vy];
xhat_future = [ball(:).XhatFuture];
yhat_future = [ball(:).YhatFuture];
corrections = nonzeros([ball(:).Correction]);
corrections_t = find([ball(:).Correction]==1)*Tcontrol;

% Create a shifted corrections to be at mean of x and y
% The '-1' is because they are initially Booleans
corrections_x = (corrections-1) + mean(xhat);
corrections_y = (corrections-1) + mean(yhat);

figure(1); clf;
ax1 = subplot(411);
plot(t,xhat,t,xhat_future,x_t,x);
legend('estimated','predicted','camera');
xlim([0 t(end)]);
title('x-position');
xlabel('time (s)');
ylabel('Position (m)');
hold on;
s = scatter(corrections_t,corrections_x, 'k');
set(s,'sizedata', .6);

ax2 = subplot(412);
plot(t,vx);
xlim([0 t(end)]);
title('x-velocity');
xlabel('time (s)');
ylabel('Velocity (m/s)');

ax3 = subplot(413);
plot(t,yhat,t,yhat_future,y_t,y);
legend('estimated','predicted','camera');
xlim([0 t(end)]);
title('y-position');
xlabel('time (s)');
ylabel('Position (m)');
hold on;
s = scatter(corrections_t,corrections_y, 'k');
set(s,'sizedata', .6);

ax4 = subplot(414);
plot(t,vy);
xlim([0 t(end)]);
title('y-velocity');
xlabel('time (s)');
ylabel('Velocity (m/s)');

linkaxes([ax1, ax2, ax3, ax4], 'x');

% Plot the initial position