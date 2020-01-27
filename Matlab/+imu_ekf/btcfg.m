function btcfg(port)
%BTCFG() Configures HC06 Bluetooth module
%   
%   Inputs:
%   - port = USB serial port [char, default = 'COM16']
%   
%   Author: Dan Oates (WPI class of 2020)

% Imports
import('serial_com.hc06_config');

% Default args
if nargin < 1, port = 'COM16'; end

% Params
name = 'IMU-EKF';
pin = '9243';
baud = 57600;

% Configure HC06
hc06_config(name, pin, baud, port);

end