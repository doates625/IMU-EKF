function [bias, cov_] = cal_gyr(log, disp_)
%[bias, cov_] = CAL_GYR(log, disp_)
%   Gyroscope calibration
%   
%   Inputs:
%   - log = Log of stationary IMU [imu_ekf.Log]
%   - disp_ = Display flag [logical, default = true]
%   
%   Outputs:
%   - bias = Bias vector [rad/s, size = [3, 1]]
%   - cov_ = Covariance matrix [(rad/s)^2, size = [3, 3]]
%   
%   The default log is from 'log_cal_gyr.mat'.

% Imports
import('imu_ekf.Log');

% Default args
if nargin < 1, log = Log('log_cal_gyr.mat'); end
if nargin < 2, disp_ = true; end

% Function
bias = mean(log.ang_vels, 2);
err = log.ang_vels - bias;
cov_ = err * err.' / log.log_len;

% Display results
if disp_
    
    % Printout
    clc
    fprintf('Gyro Calibration\n')
    fprintf('\n');
    fprintf('Biases [rad/s]:\n');
    fprintf('x: %+.7e\n', bias(1));
    fprintf('y: %+.7e\n', bias(2));
    fprintf('z: %+.7e\n', bias(3));
    fprintf('\n');
    fprintf('Covariance [(rad/s)^2]:\n');
    fprintf('[%+.7e, %+.7e, %+.7e]\n', cov_(1, :));
    fprintf('[%+.7e, %+.7e, %+.7e]\n', cov_(2, :));
    fprintf('[%+.7e, %+.7e, %+.7e]\n', cov_(3, :));
    fprintf('\n');
    
    % Plots
    close all
    make_plot(1, 'Gyro Raw', 'ro', log.ang_vels);
    make_plot(2, 'Gyro Cal', 'bo', log.ang_vels - bias);
end

function make_plot(fig_, title_, linespec, vels)
    %MAKE_PLOT(fig_, title_, linespec, vels)
    %   Make gyroscope sample plot
    %   
    %   Inputs:
    %   - fig_ = Figure number [int]
    %   - title_ = Figure title [char]
    %   - linespec = Plot linespec [char]
    %   - vels = Samples [rad/s, size = [3, N]]
    
    % Parse velocity array
    vels_x = vels(1, :);
    vels_y = vels(2, :);
    vels_z = vels(3, :);
    
    % Generate plot
    figure(fig_);
    clf, hold on
    plot3(vels_x, vels_y, vels_z, linespec);
    
    % Format plot
    title(title_);
    xlabel('Vel-x [rad/s]')
    ylabel('Vel-y [rad/s]')
    zlabel('Vel-z [rad/s]')
    view(-30, 25)
    grid on
    camproj perspective
    axis equal
end

end