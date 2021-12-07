%% Script initialization
clc, clear, close all


%% State space model for the discrete system (no input)

% Ts = 0.2;
% A = [1, Ts, Ts^2/2;
%      0, 1, Ts;
%      0, 0, 1]; 
% C = [0, 1, 0;
%      0, 0, 1];
% Q = [0.1 , 0, 0;
%      0, 0.15, 0;
%      0, 0, 0.05];
%  
% R = [0.05, 0;
%      0, 0.1000];
%  
%  
%  
% x_est(:,1) = [0;10;0];
% x_est_apriori(:,1) = [0;10;0];
% x_est_prev = [0;10;2];
% P_est_prev = eye(3);
%  
% 
% measured_speed = 5000;
% measured_accel = 2;
% 
% for i=2:50
%     [x_est(:,i), P_est, x_est_apriori(:,i)] = kalman_filter_1D(x_est_prev, P_est_prev, measured_speed, measured_accel, A, C, Q, R);
%     P_est_prev = P_est;
%     x_est_prev = x_est(:,i);
% end
% 
% plot(x_est(1,:));
% hold on
% %plot(x_est_apriori(1,:));


%%
vision = 0;
N = 8;


Ts = 0.2;

A = [1, Ts, Ts^2/2, 0, 0, 0, 0, 0;
     0, 1, Ts, 0, 0, 0, 0, 0;
     0, 0, 1, 0, 0, 0, 0, 0;
     0, 0, 0, 1, Ts, Ts^2/2, 0, 0;
     0, 0, 0, 0, 1, Ts, 0, 0;
     0, 0, 0, 0, 0, 1, 0, 0;
     0, 0, 0, 0, 0, 0, 1, Ts
     0, 0, 0, 0, 0, 0, 0, 1]; 
 
Q = 0.1*eye(N);

switch vision
    case 1
      C = [];
    case 0
        M = 5;
        C = [0, 1, 0, 0, 0, 0, 0, 0;
             0, 0, 0, 0, 1, 0, 0, 0;
             0, 0, 0, 0, 0, 0, 0, 1;
             0, 0, 1, 0, 0, 0, 0, 0;
             0, 0, 0, 0, 0, 1, 0, 0];
        R = 0.2*eye(M);
         
end

x_est(:,1) = [20;5;0;50;0;0;0;0];
x_est_apriori(:,1) = x_est(:,1);
x_est_prev = [19;5;0;20;0;0;0;0];
P_est_prev = eye(8);
 

speed_measure = [100; 20; 0];
accel_measure = [0; 0];

for i=2:50
    [x_est(:,i), P_est, x_est_apriori(:,i)] = kalman_filter_1D(x_est_prev, P_est_prev, speed_measure, accel_measure, A, C, Q, R);
    P_est_prev = P_est;
    x_est_prev = x_est(:,i);
end

figure(1);
hold on
plot(x_est(1,:), x_est(4,:));
plot(x_est_apriori(1,:), x_est_apriori(4,:));
legend('after','before')

hold off

