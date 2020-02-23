classdef EKF < kalman.EKF
    %EKF IMU Extended Kalman Filter
    %   
    %   State x:
    %   - q_e = Attitude Earth [quat]
    %   - b_e = Magnetic field Earth [uT]
    %   
    %   Input u:
    %   - w_b = Angular velocity Body [rad/s]
    %   
    %   Output z:
    %   - b_b = Magnetic field Body [uT]
    
    methods (Access = public, Static)
        function x = pack(q_e, b_e)
            %x = PACK(q_e, b_e)
            %   Pack state vector
            %   - q_e = Attitude Earth [quat]
            %   - b_e = Mag field Earth [uT]
            %   - x = State vector [q_e; b_e]
            x = [q_e; b_e];
        end
        
        function [q_e, b_e] = unpack(x)
            %[q_e, b_e] = UNPACK(x)
            %   Unpack state vector
            %   - x = State vector [q_e; b_e]
            %   - q_e = Attitude Earth [quat]
            %   - b_e = Mag field Earth [uT]
            q_e = x(1:4);
            b_e = x(5:7);
        end
    end
    
    methods (Access = public)
        function obj = EKF(q_est, b_est, cov_q, cov_w, cov_b, del_t)
            %obj = EKF(q_est, b_est, cov_q, cov_w, cov_b, del_t)
            %   Construct IMU EKF
            %   - q_est = Attitude estimate [quat]
            %   - b_est = Mag field Earth estimate [uT]
            %   - cov_q = Attitude cov [quat^2]
            %   - cov_w = Angular vel Body cov [(rad/s)^2]
            %   - cov_b = Mag field Body cov [uT^2]
            %   - del_t = Prediction time delta [s]
            
            % Imports
            import('imu_ekf.EKF.pack');
            
            % State estimate
            x_est = pack(q_est, b_est);
            cov_x = zeros(7);
            cov_x(1:4, 1:4) = cov_q;
            cov_x(5:7, 5:7) = cov_b;
            
            % Functions
            f = @(x, u) imu_ekf.EKF.f_(x, u, del_t);
            h = @(x) imu_ekf.EKF.h_(x);
            fx = @(x, u) imu_ekf.EKF.fx_(x, u, del_t);
            fu = @(x, u) imu_ekf.EKF.fu_(x, u, del_t);
            hx = @(x) imu_ekf.EKF.hx_(x);
            
            % Superconstructor
            obj@kalman.EKF(x_est, cov_x, cov_w, cov_b, f, h, fx, fu, hx);
        end
        
        function x_est = correct(obj, z)
            %x_est = CORRECT(obj, z)
            %   Correction step
            %   - z = Output vector [b_b]
            %   - x_est = Corrected state [q_e; b_e]
            
            % Imports
            import('imu_ekf.EKF.unpack');
            
            % EKF correction
            x_est = correct@kalman.EKF(obj, z);
            
            % Quaternion normalization
            [q_e, ~] = unpack(x_est);
            N = ones(7, 1);
            N(1:4) = 1 / norm(q_e);
            N = diag(N);
            obj.x_est = N * x_est;
            obj.cov_x = N * obj.cov_x * N;
            x_est = obj.x_est;
        end
    end
    
    methods (Access = protected, Static)        
        function xn = f_(x, u, del_t)
            %xn = F_(x, u, del_t)
            %   State transition function
            %   - x = State vector [q_e; b_e]
            %   - u = Input vector [w_b]
            %   - del_t = Time delta [s]
            %   - xn = Next state [q_e; b_e]

            % Imports
            import('imu_ekf.EKF.unpack');
            import('imu_ekf.EKF.pack');
            import('quat.Quat');
            
            % Unpack x and u
            [q_e, b_e] = unpack(x);
            w_b = u;

            % Attitude update
            del_q = Quat(w_b, norm(w_b) * del_t);
            q_e = Quat(q_e) * del_q;

            % Re-pack x
            q_e = q_e.vector();
            xn = pack(q_e, b_e);
        end
        
        function z = h_(x)
            %z = H_(x)
            %   Output function
            %   - x = State vector [q_e; b_e]
            %   - z = Output vector [b_b]
            
            % Imports
            import('imu_ekf.EKF.unpack');
            import('quat.Quat');
            
            % Magnetometer output
            [q_e, b_e] = unpack(x);
            z = Quat(q_e).inv().rotate(b_e);
        end
        
        function jac_xx = fx_(~, u, del_t)
            %jac_xx = FX_(x, u, del_t)
            %   Get state Jacobian
            %   - x = State vector [q_e; b_e]
            %   - u = Input vector [w_b]
            %   - del_t = Time delta [s]
            %   - jac_xx = State Jacobian

            % Imports
            import('imu_ekf.EKF.unpack');
            import('quat.Quat');

            % Jacobian
            jac_xx = eye(7);
            jac_xx(1:4, 1:4) = Quat(u, norm(u) * del_t).mat_int();
        end
        
        function jac_xu = fu_(x, u, del_t)
            %jac_xu = FU_(x, u, del_t)
            %   Get input Jacobian
            %   - x = State vector [q_e; b_e]
            %   - u = Input vector [w_b]
            %   - del_t = Time delta [s]
            %   - jac_xu = State Jacobian

            % Imports
            import('imu_ekf.EKF.unpack');
            import('quat.Quat');

            % Unpack x and u
            [q_e, ~] = unpack(x);
            w_b = u;

            % Attitude from delta-q
            q_e = Quat(q_e);
            jac_qe_dq = q_e.mat_ext();

            % Delta-q from theta-b
            norm_wb = norm(w_b);
            norm_wb_inv = 1 / norm_wb;
            del_th = 0.5 * norm_wb * del_t;
            sin_th = sin(del_th);
            cos_th = cos(del_th);
            wh = w_b * norm_wb_inv;
            jac_dq_thb = zeros(4);
            jac_dq_thb(1, 1) = -0.5 * sin_th;
            jac_dq_thb(2:4, 1) = 0.5 * cos_th * wh;
            jac_dq_thb(2:4, 2:4) = sin_th * eye(3);

            % Theta-b from w_b
            jac_th_wb = norm_wb_inv * (w_b.' * del_t);
            jac_wh_wb = norm_wb_inv * (eye(3) - wh * wh.');
            jac_thb_wb = [jac_th_wb; jac_wh_wb];

            % Jacobian
            jac_xu = zeros(7, 3);
            jac_xu(1:4, :) = jac_qe_dq * jac_dq_thb * jac_thb_wb;
        end
        
        function jac_zx = hx_(x)
            %jac_zx = HX_(x)
            %   Get magnetometer output Jacobian
            %   - x = State vector [q_e; b_e]
            %   - jac_zx = Output Jacobian
            
            % Imports
            import('imu_ekf.EKF.unpack');
            import('quat.Quat');
            
            % Unpack state vector
            [q_e, b_e] = unpack(x);
            q_e_inv = Quat(q_e).inv();
            
            % Output Jacobain
            jac_zx = zeros(3, 7);
            jac_zx(:, 1:4) = q_e_inv.jac_rot(b_e);
            jac_zx(:, 5:7) = q_e_inv.mat_rot();
        end
    end
end