% clear; clc
rerun = 0;
% load('data/ally11.mat');

Tcamera=1/30;Tcontrol=1/100;
% cam_latency = 130E-3; % s
% update_type = 'SIMPLE';
% Q = diag([(5E-2)^2 (5E-2)^2 (10*pi/180)^2]);
% R = diag([ 0.001^2 0.001^2 (1*pi/180)^2 ]);
save('data/ally14.mat');

% How many samples are there?
N = length(bot.Xhat);

% Create a time vector
t = (1:N)*Tcontrol;

fprintf('Processing %f seconds of bot data.\r\n\r\n', N*Tcontrol);

% Initialize and unpackage
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
ax(1) = subplot(311);
plot(t,xhat,t,xhat_future,x_t,x);
if rerun
    hold on;
    plot(t,xhat_rerun);
    plot(t,xhat_future_rerun);
    hold off
end
legend('estimated','predicted','camera','rerun','predicted rerun');
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

ax(3) = subplot(312);
plot(t,yhat,t,yhat_future,y_t,y);
if rerun
    hold on;
    plot(t,yhat_rerun);
    plot(t,yhat_future_rerun);
    hold off
end
legend('estimated','predicted','camera','rerun','predicted rerun');
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

ax(5) = subplot(313);
plot(t,thetahat,t,thetahat_future,theta_t,theta);
if rerun
    hold on;
    plot(t,thetahat_rerun);
    plot(t,thetahat_future_rerun);
    hold off
end
legend('estimated','predicted','camera','rerun','predicted rerun');
xlim([0 t(end)]);
title('theta');
xlabel('time (s)');
ylabel('Angle (deg)');
hold on;
s = scatter(corrections_t,corrections_theta, 'k');
set(s,'sizedata', .6);

linkaxes(ax(:), 'x');

% Plot the initial position