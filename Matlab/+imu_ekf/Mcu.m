classdef Mcu < handle
    %MCU Bluetooth interface for embedded MCU
    %   
    %   Author: Dan Oates (WPI Class of 2020)
    
    properties (Access = protected, Constant)
        start_byte = hex2dec('FF');     % Msg start byte [uint8]
        id_start = hex2dec('F0');   % Start msg ID [uint8]
        id_stop = hex2dec('F1');    % Stop msg ID [uint8]
        id_state = hex2dec('F2');   % State msg ID [uint8]
        id_data = hex2dec('F3');    % Data msg ID [uint8]
    end
    
    properties (Access = protected)
        server; % Serial server [SerialServer]
    end
    
    properties (SetAccess = protected)
        ang_vel = [0; 0; 0];    % Bogy-fixed angular velocity [rad/s]
        mag_fld = [0; 0; 0];    % Body-fixed magnetic field [uT]
        state = 'Unknown';      % Device state enum [char]
        got_state = false;      % Received state flag [logical]
        got_data = false;       % Received data falg [logical]
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
            obj.server.add_tx(obj.id_start, 0, @obj.tx_empty);
            obj.server.add_tx(obj.id_stop, 0, @obj.tx_empty);
            obj.server.add_rx(obj.id_state, 1, @obj.rx_state);
            obj.server.add_rx(obj.id_data, 24, @obj.rx_data);
        end
        
        function start(obj)
            %START(obj) Starts periodic sampling
            obj.server.tx(obj.id_start);
        end
        
        function stop(obj)
            %STOP(obj) Stops periodic sampling
            obj.server.tx(obj.id_stop);
        end

        function update(obj)
            %UPDATE(obj) Processes incoming messages
            obj.got_state = false;
            obj.got_data = false;
            obj.server.rx();
        end
    end
    
    methods (Access = protected)
        function tx_empty(~, server)
            %TX_EMPTY(obj, server) Empty TX callback
            %   
            %   Inputs:
            %   - server = Server [SerialServer]
            %   
            %   Data format:
            %   - Empty
            server.set_tx_data([]);
        end
        
        function rx_state(obj, server)
            %RX_STATE(obj, server) State RX callback
            %   
            %   Inputs:
            %   - server = Server [SerialServer]
            %   
            %   Data format:
            %   - State byte [uint8]
            %       0x00 = Idle
            %       0x01 = Sampling
            
            % Imports
            import('serial_com.Struct');
            
            % Unpack data
            str = Struct(server.get_rx_data());
            byte = str.get('uint8');
            
            % Convert to state string
            switch byte
                case 0, obj.state = 'Idle';
                case 1, obj.state = 'Sampling';
                otherwise, error('Invalid state byte: %u', data);
            end
            
            % Set RX flag
            obj.got_state = true;
        end
        
        function rx_data(obj, server)
            %RX_DATA(obj, server) Sensor data RX callback
            %   
            %   Inputs:
            %   - server = Server [SerialServer]
            %   
            %   Data format:
            %   - Angular velocity [single, [x; y; z]]
            %   - Magnetic field [single, [x; y; z]]
            
            % Imports
            import('serial_com.Struct');
            
            % Unpack data
            str = Struct(server.get_rx_data());
            for i = 1:3, obj.ang_vel(i) = str.get('single'); end
            for i = 1:3, obj.mag_fld(i) = str.get('single'); end
            
            % Set RX flag
            obj.got_data = true;
        end
    end
end