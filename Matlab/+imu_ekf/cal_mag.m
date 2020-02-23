function [bias, cov_] = cal_mag(log, gyr_bias, disp_)
%[bias, cov_] = CAL_MAG(log, gyr_bias, disp_)
%   Magnetometer calibration
%   
%   Inputs:
%   - log = Log of rotating IMU [imu_ekf.Log]
%   - gyr_bias = Gyroscope bias vector [rad/s, size = [3, 1]]
%   - disp_ = Display flag [logical, default = true]
%   
%   Outputs:
%   - bias = Bias vector [uT, size = [3, 1]]
%   - cov_ = Covariance matrix [(uT)^2, size = [3, 3]]
%   
%   The default log is from 'log_cal_mag.mat'.
%   The default bias is determined from 'log_cal_gyr.mat'.

% Imports
import('imu_ekf.Log');
import('imu_ekf.cal_gyr');
import('quat.Quat');

% Default args
if nargin < 1, log = Log('log_cal_mag.mat'); end
if nargin < 2, gyr_bias = cal_gyr(Log('log_cal_gyr.mat'), false); end
if nargin < 3, disp_ = true; end

% Estimate orientation
n = log.log_len;
Reb = repmat(eye(3), n, 1);
q_e = Quat();
dt = log.get_dt();
for k = 1:n-1
    w_b = log.ang_vels(:, k) - gyr_bias;
    q_e = q_e * Quat(w_b, norm(w_b)*dt);
    i = 3*(k-1) + (1:3);
    Reb(i+3, :) = q_e.inv().mat_rot();
end

% Estimate bias
% A = [I3, Rtb(1); I3, Rtb(2); ...]
% x = [bias; glob]
% y = [mag_flds(1); mag_flds(2); ...]
A = [repmat(eye(3), n, 1), Reb];
y = reshape(log.mag_flds, [3*n, 1]);
x = A\y;
bias = x(1:3);
glob = x(4:6);

% Estimate covariance
mag_exp = zeros(3, n);
for k = 1:n
    i = 3*(k-1) + (1:3);
    mag_exp(:, k) = Reb(i, :) * glob + bias;
end
err = log.mag_flds - mag_exp;
cov_ = err * err.' / log.log_len;

% Display results
if disp_
    
    % Print results
    clc
    fprintf('Mag Calibration:\n');
    fprintf('\n');
    fprintf('Biases [uT]:\n')
    fprintf('x: %+.2e\n', bias(1));
    fprintf('y: %+.2e\n', bias(2));
    fprintf('z: %+.2e\n', bias(3));
    fprintf('\n');
    fprintf('Covariance [(uT)^2]:\n');
    fprintf('[%+.7e, %+.7e, %+.7e]\n', cov_(1, :));
    fprintf('[%+.7e, %+.7e, %+.7e]\n', cov_(2, :));
    fprintf('[%+.7e, %+.7e, %+.7e]\n', cov_(3, :));
    fprintf('\n');
    
    % Plot results
    close all
    make_plot(1, 'Mag Raw', 'ro', log.mag_flds);
    make_plot(2, 'Mag Cal', 'bo', log.mag_flds - bias);
end

function make_plot(fig_, title_, linespec, flds)
    %MAKE_PLOT(fig_, title_, linespec, vels)
    %   Make magnetometer plot
    %   
    %   Inputs:
    %   - fig_ = Figure number [int]
    %   - title_ = Figure title [char]
    %   - linespec = Plot linespec [char]
    %   - flds = Samples [uT, size = [3, N]]
    
    % Parse velocity array
    flds_x = flds(1, :);
    flds_y = flds(2, :);
    flds_z = flds(3, :);
    
    % Generate plot
    figure(fig_);
    clf, hold on
    plot3(flds_x, flds_y, flds_z, linespec);
    
    % Format plot
    title(title_);
    xlabel('Field-x [uT]')
    ylabel('Field-y [uT]')
    zlabel('Field-z [uT]')
    view(-30, 25)
    grid on
    camproj perspective
    axis equal
end

end