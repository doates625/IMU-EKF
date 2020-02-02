classdef EKF < handle
    %EKF Extended Kalman filter for IMU attitude tracking
    %   
    %   Notation:
    %   - q = Quaternion coordinate
    %   - w = Angular velocity [rad/s]
    %   - b = Magnetic field [uT]
    %   - v_E = Vector in Earth-frame
    %   - v_B = Vector in body-frame
    %   - CVM = Covariance matrix
    %   
    %   Variables:
    %   - x = State [qw; qx; qy; qz]
    %   - u = Inputs [w_Bx; w_By; w_Bz]
    %   - z = Observations [b_Bx; b_By; b_Bz]
    %   - cov_x = State CVM
    %   - cov_u = Input CVM
    %   - cov_z = Sensor CVM
    %   
    %   Author: Dan Oates (WPI class of 2020) 
    
    properties (Access = protected)
        cov_x;  % State CVM
        cov_u;  % Input CVM
        cov_z;  % Observation CVM
        del_t;  % Input time delta [s]
        b_E;    % Earth mag field [uT]
    end
    
    methods (Access = public)
        function obj = EKF(cov_x, cov_u, cov_z, del_t, b_E)
            %obj = EKF(x, cov_x, cov_u, cov_z, del_t, b_E)
            %   Construct IMU EKF
            %   
            %   Inputs:
            %   - cov_x = Initial state CVM
            %   - cov_u = Input covariance
            %   - cov_z = Observation coveriance
            %   - del_t = Input time delta [s]
            %   - b_E = Earth mag field [uT]
            obj.cov_x = cov_x;
            obj.cov_u = cov_u;
            obj.cov_z = cov_z;
            obj.del_t = del_t;
            obj.b_E = b_E;
        end
        
        function [x, cov_x] = predict(obj, x, cov_x, u)
            %[x, cov_x] = PREDICT(obj, x, cov_x, u, del_t)
            %   EKF prediction step
            %   
            %   Inputs:
            %   - x = Current state
            %   - cov_x = Current state CVM
            %   - u = Input vector
            %   
            %   Outputs:
            %   - x = Predicted state
            %   - cov_x = Predicted state CVM
            
            % Imports
            import('quat.Quat');
            
            % Convert u to axis-angle
            norm_u = norm(u);
            norm_u_inv = 1/norm_u;
            ang = obj.del_t * norm_u;
            u_h = u * norm_u_inv;
            del_ut_u = norm_u_inv * [obj.del_t*u.'; eye(3) - u_h*u_h.'];
            
            % Convert u to quaternion
            ang_h = 0.5 * ang;
            c_ah = cos(ang_h);
            s_ah = sin(ang_h);
            u_q = [c_ah; s_ah*u_h];
            del_uq_ut = [-0.5*s_ah, zeros(1,3); 0.5*c_ah*u_h, s_ah*eye(3)];
            
            % Prediction
            x_q = Quat(x);
            u_q = Quat(u_q);
            x = x_q * u_q;
            x = x.vector();
            
            % Prediction covariance
            Fx = u_q.mat_int();
            Fu = x_q.mat_ext() * del_uq_ut * del_ut_u;
            cov_x = Fx*cov_x*Fx.' + Fu*obj.cov_u*Fu.';
        end
        
        function [x, cov_x] = correct(obj, x, cov_x, z)
            %CORRECT(obj, z)
            %   EKF correction step
            %   
            %   Inputs:
            %   - x = Predicted state
            %   - cov_x = Predicted state CVM
            %   - z = Observation vector
            %   
            %   Outputs:
            %   - x = Corrected state
            %   - cov_x = Corrected state CVM
            
            % Imports
            import('quat.Quat');
            
            % Observation jacobian
            bE = obj.b_E;
            q0 = x(1:1);
            q1 = x(2:4);
            H0 = 2*(q0*bE - q1);
            H1 = 2*((dot(q1, bE) - q0)*eye(3) + q1*bE.' - bE*q1.');
            H = [H0, H1];
            
            % State correction
            hx = Quat(x).inv().rotate(bE);
            K = (cov_x*H.')/(H*cov_x*H.' + obj.cov_z);
            x = x + K*(z - hx);
            cov_x = (eye(4) - K*H)*cov_x;
        end
    end
end

