function usb_debug(port, baud)
%USB_DEBUG(port) Prints USB debug from MCU
%   
%   Inputs:
%   - port = Port name [char, default = 'COM17']
%   - baud = Baud rate [int, default = 115200]
%   
%   Author: Dan Oates (WPI Class of 2020)

% Imports
import('serial_com.make_serial');
import('serial_com.terminal');

% Default args
if nargin < 1, port = 'COM17'; end
if nargin < 2, baud = 115200; end

% Serial terminal
instrreset;
ser = make_serial(port, baud);
terminal(ser);

end