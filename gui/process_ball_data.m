% clear; close all; clc;
% load('ball_msg_data.mat');

% How many samples are there?
N = length(ball);

% Create a time vector
t = (0:N-1)*Tcontrol;

fprintf('Processing %f seconds of ball data.\r\n\r\n', N*Tcontrol);

% Initialize
xhat = zeros(1,N);
yhat = zeros(1,N);
vx = zeros(1,N);
vy = zeros(1,N);
xhat_future = zeros(1,N);
yhat_future = zeros(1,N);

% Unpackage
for i = 1:N
    xhat(i) = ball(i).Xhat;
    yhat(i) = ball(i).Yhat;
    vx(i) = 0;
    vy(i) = 0;
    xhat_future(i) = ball(i).XhatFuture;
    yhat_future(i) = ball(i).YhatFuture; 
end

figure(1), clf;
subplot(211);
plot(t,xhat,t,xhat_future);
xlim([0 t(end)]);
title('xhat vs xhat\_future');
xlabel('time (s)');
ylabel('Position (m)');

subplot(212);
plot(t,yhat,t,yhat_future);
xlim([0 t(end)]);
title('yhat vs yhat\_future');
xlabel('time (s)');
ylabel('Position (m)');

% Plot the initial position