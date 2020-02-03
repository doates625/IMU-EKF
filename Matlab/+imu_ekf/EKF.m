classdef (Abstract) EKF < handle
    %EKF Superclass for EKF Matlab algorithms
    %   
    %   Notation:
    %   - x = State vector
    %   - u = Input vector
    %   - z = Observation vector
    %   - q = Quaternion 
    %   - w = Angular velocity [rad/s]
    %   - b = Magnetic field [uT]
    %   - vE = Vector in Earth frame
    %   - vB = Vector in Body frame
    %   - cov_v = Covariance matrix of v
    %   
    %   Author: Dan Oates (WPI Class of 2020)
    
    properties (Access = protected)
        cov_u;  % Input CVM
        cov_z;  % Observation CVM
        del_t;  % Prediction time delta [s]
    end
    
    methods (Access = public)
        function obj = EKF(cov_u, cov_z, del_t)
            %EKF(cov_u, cov_z, del_t)
            %   Construct EKF algorithm
            %   
            %   Inputs:
            %   - cov_u = Input cov
            %   - cov_z = Observation cov
            %   - del_t = Time delta [s]
            obj.cov_u = cov_u;
            obj.cov_z = cov_z;
            obj.del_t = del_t;
        end
        
        function [x, cov_x] = predict(obj, x, cov_x, u)
            %[x, cov_x] = PREDICT(obj, x, cov_x, u)
            %   EKF prediction step with quaternion normalization
            %   
            %   Inputs:
            %   - x = State estimate
            %   - cov_x = State CVM
            %   - u = Input vector
            %   
            %   Outputs:
            %   - x = Predicted state
            %   - cov_x = Predicted state CVM
            [x, cov_x] = obj.predict_(x, cov_x, u);
            [x, cov_x] = obj.normalize(x, cov_x);
        end
        
        function [x, cov_x] = correct(obj, x, cov_x, z)
            %[x, cov_x] = CORRECT(obj, x, cov_x, z)
            %   EKF correction step with quaternion normalization
            %   
            %   Inputs:
            %   - x = State estimate
            %   - cov_x = State CVM
            %   - z = Observation vector
            %   
            %   Outputs:
            %   - x = Corrected state
            %   - cov_x = Corrected state CVM
            [x, cov_x] = obj.correct_(x, cov_x, z);
            [x, cov_x] = obj.normalize(x, cov_x);
        end
    end
    
    methods (Access = protected, Abstract)
        [x, cov_x] = predict_(obj, x, cov_x, u)
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
        
        [x, cov_x] = correct_(obj, x, cov_x, z)
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
    end
    
    methods (Access = protected)
        function [q, del_q_q, del_q_w] = predict_q(obj, q, w)
            %[q, del_q_q, del_q_w] = PREDICT_Q(obj, q, w)
            %   Quaternion prediction step
            %   
            %   Inputs:
            %   - q = Quaternion orientation
            %   - w = Body-fixed angular velocity [rad/s]
            %   
            %   Outputs:
            %   - q = Predicted orientation
            %   - del_q_q = Quaternion jacobian
            %   - del_q_w = Angular velocity jacobian
            
            % Imports
            import('quat.Quat');
            
            % Convert to axis-angle
            norm_w = norm(w);
            norm_w_inv = 1/norm_w;
            ang = obj.del_t * norm_w;
            wh = w * norm_w_inv;
            del_aa_w = norm_w_inv * [obj.del_t*w.'; eye(3) - wh*wh.'];
            
            % Convert to quaternion
            ang_h = 0.5 * ang;
            c_ah = cos(ang_h);
            s_ah = sin(ang_h);
            dq = [c_ah; s_ah*wh];
            del_dq_aa = [-0.5*s_ah, zeros(1,3); 0.5*c_ah*wh, s_ah*eye(3)];
            
            % Update orientation
            dq = Quat(dq);
            q = Quat(q) * dq;
            del_q_dq = q.mat_ext();
            q = q.vector();
            
            % Transform jacobians
            del_q_q = dq.mat_int();
            del_q_w = del_q_dq * del_dq_aa * del_aa_w;
        end
        
        function cov_x = predict_cov_x(obj, cov_x, del_x_x, del_x_u)
            %cov_x = PREDICT_COV_X(obj, cov_x, del_x_x, del_x_u)
            %   Update state covariance from prediction step
            %   
            %   Inputs:
            %   - cov_x = State covariance
            %   - del_x_x = State jacobian
            %   - del_x_u = Input jacobian
            %   
            %   Outputs:
            %   - cov_x = Updated covariance
            cov_x_x = del_x_x * cov_x * del_x_x.';
            cov_x_u = del_x_u * obj.cov_u * del_x_u.';
            cov_x = cov_x_x + cov_x_u;
        end
        
        function [x, cov_x] = correct_ekf(obj, x, cov_x, z, z_exp, del_z_x)
            %[x, cov_x] = CORRECT_EKF(obj, x, cov_x, z, z_exp, del_z_x)
            %   EKF linearized correction step
            %   
            %   Inputs:
            %   - x = State estimate
            %   - cov_x = State covariance
            %   - z = Observation
            %   - z_exp = Predicted observation
            %   - del_z_x = Observation jacobian
            %   
            %   Outputs:
            %   - x = Corrected state estimate
            %   - cov_x = Corrected state covariance
            k_gain = (cov_x*del_z_x.')/(del_z_x*cov_x*del_z_x.' + obj.cov_z);
            x = x + k_gain*(z - z_exp);
            n = length(x);
            cov_x = (eye(n) - k_gain*del_z_x)*cov_x;
        end
    end
    
    methods (Access = protected, Static)
        function [bB, del_bB_q, del_bB_bE] = predict_bB(q, bE)
            %[bB, del_bB_q, del_bB_bE] = PREDICT_BB(obj, q, bE)
            %   Body-fixed mag field prediction
            %   
            %   Inputs:
            %   - q = Orientation estimate
            %   - bE = Global mag field estimate [uT]
            %   
            %   Outputs:
            %   - bB = Expected body-fixed field [uT]
            %   - del_bB_q = Quaternion jacobian
            %   - del_bB_bE = Global field jacobian
            
            % Imports
            import('quat.Quat');
            
            % Field prediction
            Reb = Quat(q).inv().mat_rot();
            bB = Reb * bE;
            
            % Quaternion jacobian
            q0 = q(1:1);
            q1 = q(2:4);
            del_bB_q0 = 2*(q0*bE - q1);
            del_bB_q1 = 2*((dot(q1, bE) - q0)*eye(3) + q1*bE.' - bE*q1.');
            del_bB_q = [del_bB_q0, del_bB_q1];
            
            % Global field jacobian
            del_bB_bE = Reb;
        end
        
        function [x, cov_x] = normalize(x, cov_x)
            %[x, cov_x] = NORMALIZE(x, cov_x)
            %   Normalizes quaternion of state estimate
            %   
            %   Inputs:
            %   - x = State vector
            %   - cov_x = State CVM
            %   
            %   Outputs:
            %   - x = Normalized state vector
            %   - cov_x = Normalized state CVM
            diag_A = ones(length(x), 1);
            diag_A(1:4) = 1 / norm(x(1:4));
            A = diag(diag_A);
            x = A*x;
            cov_x = A*cov_x*A.';
        end
    end
end