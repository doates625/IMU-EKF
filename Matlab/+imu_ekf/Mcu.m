classdef Mcu < handle
    %MCU Bluetooth interface for embedded MCU
    %   
    %   Author: Dan Oates (WPI Class of 2020)
    
    properties (Access = protected, Constant)
        start_byte = hex2dec('FF'); % Msg start byte [uint8]
        id_samp = hex2dec('F0');    % Sample msg ID [uint8]
        id_data = hex2dec('F1');    % Data msg ID [uint8]
    end
    
    properties (Access = protected)
        server;     % Serial server [SerialServer]
        times;      % Timestamp log [s, size = [1, N] ]
        ang_vels;   % Angular velocity log [rad/s, size = [3, N]]
        mag_flds;   % Magnetic field log [uT, size = [3, N]]
        log_len;    % Number of recorded stamps
        sampling;   % Sampling enabled flag [logical]
        got_data;   % New data flag [logical]
    end
    
    methods (Access = public)
        function obj = Mcu()
            %obj = MCU() Construct bluetooth interface
            
            % Imports
            import('serial_com.make_bluetooth');
            import('serial_com.SerialServer');
            
            % Bluetooth interface
            bt = make_bluetooth('IMU-EKF');
            obj.server = SerialServer(bt, obj.start_byte);
            obj.server.add_tx(obj.id_samp, 1, @obj.tx_samp);
            obj.server.add_rx(obj.id_data, 28, @obj.rx_data);
            
            % Set fields
            obj.times = zeros(1, 0);
            obj.ang_vels = zeros(3, 0);
            obj.mag_flds = zeros(3, 0);
            obj.log_len = 0;
            obj.sampling = false;
            obj.got_data = false;
        end
        
        function sample(obj, sampling)
            %SAMPLE(obj, sampling)
            %   Set sampling enable
            %   
            %   Inputs:
            %   - sampling = Sampling enable [logical]
            obj.sampling = sampling;
            obj.server.tx(obj.id_samp);
        end

        function [time, ang_vel, mag_fld] = update(obj)
            %[time, ang_vel, mag_fld] = update(obj)
            %   Processes incoming messages
            %   
            %   Outputs:
            %   - time = Timestamp [s]
            %   - ang_vel = Angular velocity [rad/s]
            %   - mag_fld = Magnetic field [uT]
            obj.got_data = false;
            while ~obj.got_data
                obj.server.rx();
            end
            n = obj.log_len;
            time = obj.times(n);
            ang_vel = obj.ang_vels(:, n);
            mag_fld = obj.mag_flds(:, n);
        end
        
        function delete(obj)
            %DELETE(obj) Saves data to log file
            
            % Format log file name
            time = datetime(now, 'ConvertFrom', 'datenum');
            time.Format = 'MM-dd-HH-mm';
            log_name = ['logs/Log-', char(time), '.mat'];
            
            % Extract variables
            times_ = obj.times;
            ang_vels_ = obj.ang_vels;
            mag_flds_ = obj.mag_flds;
            
            % Save to file
            save(log_name, 'times_', 'ang_vels_', 'mag_flds_');
        end
    end
    
    methods (Access = protected)
        function tx_samp(obj, server)
            %TX_SAMP(obj, server) Set sampling enable TX callback
            %   
            %   Inputs:
            %   - server = Server [SerialServer]
            %   
            %   Data format:
            %   - Sampling enable [uint8]
            %       0x00 = Disabled
            %       0x01 = Enabled
            server.set_tx_data(obj.sampling);
        end
        
        function rx_data(obj, server)
            %RX_DATA(obj, server) Sensor data RX callback
            %   
            %   Inputs:
            %   - server = Server [SerialServer]
            %   
            %   Data format:
            %   - Timestamp [single, s]
            %   - Angular velocity [single, [x; y; z]]
            %   - Magnetic field [single, [x; y; z]]
            
            % Imports
            import('serial_com.Struct');
            
            % Unpack data
            str = Struct(server.get_rx_data());
            n = obj.log_len + 1;
            obj.times(n) = str.get('single');
            for i = 1:3, obj.ang_vels(i, n) = str.get('single'); end
            for i = 1:3, obj.mag_flds(i, n) = str.get('single'); end
            obj.log_len = n;
            
            % Set RX flag
            obj.got_data = true;
        end
    end
end