classdef EKF1 < imu_ekf.EKF
    %EKF1 EKF algorithm with pre-calibration
    %   
    %   Model:
    %   x = [q_w; q_x; q_y; q_z]
    %   u = [wB_x; wB_y; wB_z]
    %   z = [bB_x; bB_y; bB_z]
    %   
    %   Author: Dan Oates (WPI class of 2020) 
    
    properties (Access = protected)
        bE; % Earth mag field [uT]
    end
    
    methods (Access = public)
        function obj = EKF1(cov_u, cov_z, del_t, bE)
            %obj = EKF1(cov_u, cov_z, del_t, bE)
            %   Construct EKF1 algorithm
            %   
            %   Inputs:
            %   - cov_u = Input covariance
            %   - cov_z = Observation coveriance
            %   - del_t = Input time delta [s]
            %   - bE = Earth mag field [uT]
            obj@imu_ekf.EKF(cov_u, cov_z, del_t);
            obj.bE = bE;
        end
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
            [x, del_x_x, del_x_u] = obj.predict_q(x, u);
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
            [z_exp, del_z_x, ~] = obj.predict_bB(x, obj.bE);
            [x, cov_x] = obj.correct_ekf(x, cov_x, z, z_exp, del_z_x);
        end
    end
end