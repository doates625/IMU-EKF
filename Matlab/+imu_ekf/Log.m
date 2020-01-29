classdef Log < imu_ekf.AbsMcu
    %LOG Embedded MCU log interface
    %   
    %   Author: Dan Oates (WPI Class of 2020)
    
    properties (SetAccess = protected)
        file;   % File name [char]
    end
    
    methods
        function obj = Log(file)
            %LOG(file)
            %   Load data from log file
            %   
            %   Inputs:
            %   - file = File name [char]
            data = load(['logs/' file]);
            times = data.times_;
            ang_vels = data.ang_vels_;
            mag_flds = data.mag_flds_;
            obj@imu_ekf.AbsMcu(times, ang_vels, mag_flds);
            obj.file = file;
        end
    end
end