function [x_est,P_est, x_est_apriori] = kalman_filter(x_est_prev,P_est_prev, speed_measure, accel_measure, A, C, Q, R)
%% Kalman filter function
% Kalman filter for estimating state of Thymio (position and speed)

%% Inputs




%% Outputs






%%


x_est_apriori = A*x_est_prev;
P_est_apriori = A*P_est_prev*A' + Q;
y_pred = C*x_est_apriori;
y_measured = [speed_measure; accel_measure];

S = C*P_est_apriori*C' + R;
K = P_est_apriori * C' * S^(-1);

x_est = x_est_apriori + K*(y_measured-y_pred);
P_est = P_est_apriori - K*C*P_est_apriori;




end

