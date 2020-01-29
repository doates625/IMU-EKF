function [gyr_bias, gyr_cov, mag_cov] = cal_gyr(mcu)
%[gyr_bias, gyr_cov, mag_cov] = CAL_GYR(mcu)
%   Gyroscope calibration
%   
%   Inputs:
%   - mcu = Mcu with stationary samples [imu_ekf.AbsMcu]
%   
%   Outputs:
%   - gyr_bias = Gyro bias vector [rad/s]
%   - gyr_cov = Gyro covariance [(rad/s)^2]
%   - mag_cov = Magnetometer covariance [uT^2]

gyr_bias = mean(mcu.ang_vels, 2);
gyr_cov = cov(mcu.ang_vels.');
mag_cov = cov(mcu.mag_flds.');

end