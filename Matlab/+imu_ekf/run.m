function run(dur)
    %RUN(dur) Runs IMU data collection
    %   
    %   Inputs:
    %   - dur = Sampling duration [s, default = 10]
    clc, instrreset;
    
    % Imports
    import('imu_ekf.Mcu');
    import('timing.Timer');
    
    % Default args
    if nargin < 1, dur = 10; end
    
    % Initial print
    fprintf('IMU Data Collection\n\n');
    
    % Init Mcu
    fprintf('Connecting to MCU...\n');
    mcu = Mcu();
    mcu.sample(true);
    
    % Comm loop
    timer = Timer();
    while ~timer.elapsed(dur)
        [time, ang_vel, mag_fld] = mcu.update();
        clc
        fprintf('IMU Data Collection\n');
        fprintf('\n');
        fprintf('Time: %.2f\n', time);
        fprintf('\n');
        fprintf('Gyro [rad/s]:\n');
        fprintf('x: %+.2f\n', ang_vel(1));
        fprintf('y: %+.2f\n', ang_vel(2));
        fprintf('z: %+.2f\n', ang_vel(3));
        fprintf('\n');
        fprintf('Mag [uT]:\n');
        fprintf('x: %+.2f\n', mag_fld(1));
        fprintf('y: %+.2f\n', mag_fld(2));
        fprintf('z: %+.2f\n', mag_fld(3));
        fprintf('\n');
    end
    
    % Deinit MCU
    mcu.sample(false);
end