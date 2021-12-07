function [x_est, P_est, x_est_apriori] = kalman_filter_1D(x_est_prev, P_est_prev, measured_speed, measured_accel, A, C, Q, R)
%Kalman filter in 1D for Thymio using camera, accelerometer and wheel
%speeds to estimate state

x_est_apriori = A*x_est_prev;
P_est_apriori = A*P_est_prev*A' + Q;
y_measured = [measured_speed; measured_accel];
y_pred = C*x_est_apriori;

i = y_measured-y_pred;
S = C*P_est_apriori*C' + R;
K = P_est_apriori * C' * S^(-1);

x_est = x_est_apriori + K*i;
P_est = P_est_apriori - K*C*P_est_apriori;

end

