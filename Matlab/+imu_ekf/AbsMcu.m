classdef (Abstract) AbsMcu < handle
    %ABSMCU Superclass for MCU interface
    %   
    %   Author: Dan Oates (WPI Class of 2020)
    
    properties (SetAccess = protected)
        times;      % Timestamp log [s, size = [1, N]]
        ang_vels;   % Angular velocity log [rad/s, size = [3, N]]
        mag_flds;   % Magnetic field log [uT, size = [3, N]]
        log_len;    % Number of recorded samples
    end
    
    methods (Access = public)
        function obj = AbsMcu(times, ang_vels, mag_flds)
            %obj = ABSMCU()
            %   Construct MCU interface
            %   
            %   Inputs:
            %   - times = Timestamp log [s, size = [1, N]]
            %   - ang_vels = Angular velocity log [rad/s, size = [3, N]]
            %   - mag_flds = Magnetic field log [uT, size = [3, N]]
            obj.times = times;
            obj.ang_vels = ang_vels;
            obj.mag_flds = mag_flds;
            obj.log_len = length(obj.times);
        end
    end
end