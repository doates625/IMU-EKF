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
    mcu.start();
    
    % Comm loop
    timer = Timer();
    while ~timer.elapsed(dur)
        mcu.update();
        if mcu.got_state || mcu.got_data
            clc
            fprintf('IMU Data Collection\n\n');
            fprintf('State: %s\n\n', mcu.state);
            fprintf('Gyro [rad/s]:\n');
            fprintf('x: %+.2f\n', mcu.ang_vel(1));
            fprintf('y: %+.2f\n', mcu.ang_vel(2));
            fprintf('z: %+.2f\n', mcu.ang_vel(3));
            fprintf('\n');
            fprintf('Mag [uT]:\n');
            fprintf('x: %+.2f\n', mcu.mag_fld(1));
            fprintf('y: %+.2f\n', mcu.mag_fld(2));
            fprintf('z: %+.2f\n', mcu.mag_fld(3));
            fprintf('\n');
        end 
    end
    
    % Deinit MCU
    mcu.stop();
    instrreset;
end