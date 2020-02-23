function test(log_ekf, log_gyr, log_mag)
%TEST(log_ekf, log_gyr, log_mag)
%   Test EKF Matlab implementation
%   
%   Inputs:
%   - log_ekf = Log for EKF testing [imu_ekf.Log]
%   - log_gyr = Log for gyro calibration [imu_ekf.Log]
%   - log_mag = Log for magnetometer calibration [imu_ekf.Log]
%   
%   Defaults:
%   - log_ekf from 'log_test_ekf.mat'
%   - log_gyr from 'log_cal_gyr.mat'
%   - log_mag from 'log_cal_mag.mat'

% Imports
import('imu_ekf.Log');
import('imu_ekf.cal_gyr');
import('imu_ekf.cal_mag');
import('imu_ekf.EKF');
import('quat.Quat');

% Default args
if nargin < 1, log_ekf = Log('log_test_ekf.mat'); end
if nargin < 2, log_gyr = Log('log_cal_gyr.mat'); end
if nargin < 3, log_mag = Log('log_cal_mag.mat'); end

% Pre-calibration
[bias_w, cov_w] = cal_gyr(log_gyr, false);
[bias_b, cov_b] = cal_mag(log_mag, bias_w, false);

% EKF initialization
q_e = Quat().vector();
cov_q = zeros(4);
b_e = log_ekf.mag_flds(:, 1) - bias_b;
x = EKF.pack(q_e, b_e);
del_t = log_ekf.get_dt();
ekf_gyr = EKF(q_e, b_e, cov_q, cov_w, cov_b, del_t);
ekf_mag = EKF(q_e, b_e, cov_q, cov_w, cov_b, del_t);

% Simulation logs
n = log_ekf.log_len;
x_gyr = [x, zeros(7, n-1)]; % State log gyro-only
x_mag = [x, zeros(7, n-1)]; % State log mag correction
x_cov_gyr = ekf_gyr.cov_x;  % Covariance gyro-only
x_cov_mag = ekf_mag.cov_x;  % Covariance mag correction
tr_cov_gyr = zeros(1, n);   % Covariance trace log gyro-only
tr_cov_mag = zeros(1, n);   % Covariance trace log mag correction

% EKF simulation
tr_cov_gyr(1) = trace(x_cov_gyr);
tr_cov_mag(1) = trace(x_cov_mag);
for i = 1:n-1
    % Input and observation
    u = log_ekf.ang_vels(:, i) - bias_w;
    z = log_ekf.mag_flds(:, i) - bias_b;
    
    % Gyro only
    ekf_gyr.predict(u);
    x_gyr(:, i+1) = ekf_gyr.x_est;
    tr_cov_gyr(i+1) = trace(ekf_gyr.cov_x);
    
    % Mag correction
    ekf_mag.predict(u);
    ekf_mag.correct(z);
    x_mag(:, i+1) = ekf_mag.x_est;
    tr_cov_mag(i+1) = trace(ekf_mag.cov_x);
end

% Plot Orientations
names = {'w', 'x', 'y', 'z'};
t = log_ekf.times;
figure(1)
clf
for i = 1:4
    subplot(2, 2, i)
    hold on, grid on
    title(['Quat-' names{i}])
    xlabel('Time [s]')
    ylabel('Quat')
    plot(t, x_gyr(i, :), 'r-')
    plot(t, x_mag(i, :), 'b-')
    legend('Gyr', 'Mag')
end

% Plot field estimates
names = {'x', 'y', 'z'};
figure(2)
clf
for i = 1:3
    subplot(3, 1, i)
    hold on, grid on
    title(['Field- ' names{i}])
    xlabel('Time [s]')
    ylabel('Field [uT]')
    plot(t, x_gyr(i+4, :), 'r-')
    plot(t, x_mag(i+4, :), 'b-')
    ylim([-50, +50])
    legend('Gyr', 'Mag')
end

% Plot Traces
figure(3)
clf, hold on, grid on
title('Covariance Traces')
xlabel('Time [s]')
ylabel('Trace')
plot(t, tr_cov_gyr, 'r-')
plot(t, tr_cov_mag, 'b-')
legend('Gyro-Only', 'Corrected');

end