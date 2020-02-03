classdef EKF2 < imu_ekf.EKF
    %EKF2 EKF with global magnetic field estimate
    %   
    %   Model:
    %   x = [q_w; q_x; q_y; q_z; bE_x; bE_y; bE_z]
    %   u = [wB_x; wB_y; wB_z]
    %   z = [bB_x; bB_y; bB_z]
    %   
    %   Author: Dan Oates (WPI class of 2020)
    
    methods (Access = public)
        %{
        function obj = EKF2(cov_u, cov_z, del_t)
            %obj = EKF2(cov_u, cov_z, del_t)
            %   Construct IMU EKF
            %   
            %   Inputs:
            %   - cov_u = Input covariance
            %   - cov_z = Observation coveriance
            %   - del_t = Input time delta [s]
            obj@imu_ekf.EKF(cov_u, cov_z, del_t);
        end
        %}
    end
    
    methods (Access = protected)
        function [x, cov_x] = predict_(obj, x, cov_x, u)
            %[x, cov_x] = PREDICT_(obj, x, cov_x, u)
            %   EKF prediction step
            %   
            %   Inputs:
            %   - x = State estimate
            %   - cov_x = State CVM
            %   - u = Input vector
            %   
            %   Outputs:
            %   - x = Predicted state
            %   - cov_x = Predicted state CVM

            % Prediction
            q = x(1:4);
            [q, del_q_q, del_q_u] = obj.predict_q(q, u);
            x(1:4) = q;
            
            % Jacobians
            del_q_bE = zeros(4, 3);
            del_bE_q = zeros(3, 4);
            del_bE_bE = eye(3);
            del_x_x = [del_q_q, del_q_bE; del_bE_q, del_bE_bE];
            del_bE_u = zeros(3, 3);
            del_x_u = [del_q_u; del_bE_u];
            
            % Covariance
            cov_x = obj.predict_cov_x(cov_x, del_x_x, del_x_u);
        end
        
        function [x, cov_x] = correct_(obj, x, cov_x, z)
            %[x, cov_x] = CORRECT_(obj, x, cov_x, z)
            %   EKF correction step
            %   
            %   Inputs:
            %   - x = State estimate
            %   - cov_x = State CVM
            %   - z = Observation vector
            %   
            %   Outputs:
            %   - x = Corrected state
            %   - cov_x = Corrected state CVM
            q = x(1:4);
            bE = x(5:7);
            [z_exp, del_z_q, del_z_bE] = obj.predict_bB(q, bE);
            del_z_x = [del_z_q, del_z_bE];
            [x, cov_x] = obj.correct_ekf(x, cov_x, z, z_exp, del_z_x);
        end
    end
end

