%% Script initialization
clc, clear, close all


%% State space model for the discrete system (no input)

Ts = 0.2;
A = [1, Ts, Ts^2/2;
     0, 1, Ts;
     0, 0, 1]; 
C = [0, 1, 0;
     0, 0, 1];
Q = [0.1 , 0, 0;
     0, 0.15, 0;
     0, 0, 0.05];
 
R = [0.05, 0;
     0, 0.1000];
 
 
 
x_est(:,1) = [0;10;0];
x_est_apriori(:,1) = [0;10;0];
x_est_prev = [0;10;2];
P_est_prev = eye(3);
 

measured_speed = 5000;
measured_accel = 2;

for i=2:50
    [x_est(:,i), P_est, x_est_apriori(:,i)] = kalman_filter_1D(x_est_prev, P_est_prev, measured_speed, measured_accel, A, C, Q, R);
    P_est_prev = P_est;
    x_est_prev = x_est(:,i);
end

plot(x_est(1,:));
hold on
%plot(x_est_apriori(1,:));
