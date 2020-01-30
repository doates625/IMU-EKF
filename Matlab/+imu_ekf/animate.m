function animate(log, gyr_bias, mag_bias)
    %ANIMATE(log, gyr_bias, mag_bias)
    %   Animate IMU log file
    %   
    %   Inputs:
    %   - log = IMU log file [imu_ekf.Log]
    %   - gyr_bias = Gyro bias vector [rad/s]
    %   - mag_bias = Magnetometer bias vector [uT]
    %   
    %   Defaults:
    %   - log: Loaded from 'log_cal_mag.mat'
    %   - gyr_bias: Cal from 'log_cal_gyr.mat'
    %   - mag_bias: Cal from 'log_cal_mag.mat'
    
    % Imports
    import('imu_ekf.Log');
    import('imu_ekf.cal_gyr');
    import('imu_ekf.cal_mag');
    import('live_plot.Frame3D');
    import('live_plot.Vector3D');
    import('quat.Quat');
    import('timing.Timer');
    
    % Default args
    if nargin < 1
        log = Log('log_cal_mag.mat');
    end
    if nargin < 2
        log_gyr = Log('log_cal_gyr.mat');
        gyr_bias = cal_gyr(log_gyr, false);
    end
    if nargin < 3
        log_mag = Log('log_cal_mag.mat');
        mag_bias = cal_mag(log_mag, gyr_bias, false);
    end
    
    % Format figure
    figure(1)
    clf, hold on, grid on
    title('IMU Attitude')
    xlabel('x')
    ylabel('y')
    zlabel('z')
    plot_att = Frame3D(30);
    plot_att.plot_x.plot_.Color = 'r';
    plot_att.plot_y.plot_.Color = 'g';
    plot_att.plot_z.plot_.Color = 'b';
    plot_att.update(eye(3));
    plot_mag = Vector3D();
    plot_mag.plot_.Color = 'k';
    plot_mag.update([0; 0; 0]);
    legend('att-x', 'att-y', 'att-z', 'mag')
    axis equal
    camproj perspective
    xlim([-50, +50]);
    ylim([-50, +50]);
    zlim([-50, +50]);
    view(-30, 20);
    
    % Run animation
    att = Quat();
    dt = log.get_dt();
    timer = Timer();
    for k = 1:log.log_len
        % Estimate global mag field
        rot = att.mat_rot();
        mag = rot*(log.mag_flds(:, k) - mag_bias);
        
        % Update plots
        plot_att.update(rot);
        plot_mag.update(mag);
        drawnow
        
        % Integrate attitude
        ang_vel = log.ang_vels(:, k) - gyr_bias;
        att = att * Quat(ang_vel, norm(ang_vel)*dt);
        
        % Loop timing
        timer.wait(dt);
        timer.tic();
    end
end