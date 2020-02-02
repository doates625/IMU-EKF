function test(log_ekf, log_gyr, log_mag)
%TEST(log_ekf, log_gyr, log_mag)
%   Test IMU EKF Matlab implementation
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

% Default args
if nargin < 1, log_ekf = Log('log_test_ekf.mat'); end
if nargin < 2, log_gyr = Log('log_cal_gyr.mat'); end
if nargin < 3, log_mag = Log('log_cal_mag.mat'); end

% Pre-calibration
[bias_gyr, cov_gyr] = cal_gyr(log_gyr, false);
[bias_mag, cov_mag] = cal_mag(log_mag, bias_gyr, false);

% EKF initialization
x = [1; 0; 0; 0];
cov_x = 0.01*eye(4);
cov_u = cov_gyr;
cov_z = cov_mag;
del_t = log_ekf.get_dt();
b_E = log_ekf.mag_flds(:, 1) - bias_mag;
ekf = EKF(cov_x, cov_u, cov_z, del_t, b_E);

% Simulation logs
n = log_ekf.log_len;
x_gyr = [x, zeros(4, n-1)]; % State log gyro-only
x_mag = [x, zeros(4, n-1)]; % State log mag correction
x_gyr_cov = cov_x;          % Covariance gyro-only
x_mag_cov = cov_x;          % Covariance mag correction
tr_cov_gyr = zeros(1, n);   % Covariance trace log gyro-only
tr_cov_mag = zeros(1, n);   % Covariance trace log mag correction

% EKF simulation
tr_cov_gyr(1) = trace(x_gyr_cov);
tr_cov_mag(1) = trace(x_mag_cov);
for i = 1:n-1
    % Input and observation
    u = log_ekf.ang_vels(:, i) - bias_gyr;
    z = log_ekf.mag_flds(:, i) - bias_mag;
    
    % Gyro only
    x = x_gyr(:, i);
    cov_x = x_gyr_cov;
    [x, cov_x] = ekf.predict(x, cov_x, u);
    x_gyr(:, i+1) = x;
    x_gyr_cov = cov_x;
    tr_cov_gyr(i+1) = trace(x_gyr_cov);
    
    % Mag correction
    x = x_mag(:, i);
    cov_x = x_mag_cov;
    [x, cov_x] = ekf.predict(x, cov_x, u);
    [x, cov_x] = ekf.correct(x, cov_x, z);
    x_mag(:, i+1) = x;
    x_mag_cov = cov_x;
    tr_cov_mag(i+1) = trace(x_mag_cov);
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

% Plot Traces
figure(2)
clf, hold on, grid on
title('Covariance Traces')
xlabel('Time [s]')
ylabel('Trace')
plot(t, tr_cov_gyr, 'r-')
plot(t, tr_cov_mag, 'b-')
legend('Gyro-Only', 'Corrected');

end