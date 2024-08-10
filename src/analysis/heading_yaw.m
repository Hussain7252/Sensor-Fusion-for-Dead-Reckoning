%% Calculating the magnetometer calibration matrix

% Loading the imu data taken in circle
load('imucircle1.mat')

% Extracting the magnetic field data
mag_data = imucircleimu1(936:4744,27:28);
mag_data = table2array(mag_data);

% Fitting the ellipse in the uncalibrated data using fit_ellipse function
mag_cal = fit_ellipse(mag_data(:,1),mag_data(:,2));
title("Uncorrected magnetometer data")


% Calculating the transformation matrices
mt = [mag_cal.X0_in; mag_cal.Y0_in];
mrot = [cos(mag_cal.phi), -sin(mag_cal.phi); sin(mag_cal.phi), cos(mag_cal.phi)];
scale = mag_cal.short_axis/mag_cal.long_axis;
ms = [scale,0;0,1];
translation = mag_data(:,:).'-[mt(1,1)*ones(1,size(mag_data,1));mt(2,1)*ones(1,size(mag_data,1))];
mag_data_cal = ms*mrot*translation;

% Fitting the circle in the calibrated data
hold on
fit_ellipse(mag_data_cal(1,:).',mag_data_cal(2,:).');
title("Corrected magnetometer data")

fprintf("\nPrinting the translation matrix:\n")
display(mt)
fprintf("\nPrinting the rotational matrix:\n")
display(mrot)
fprintf("\nPrinting the scaling matrix:\n")
display(ms)

%% Calculating magnetometer yaw

% Loading the imu data taken in circle
load('imudriving1.mat');

% Extracting the magnetic field data
mag_driving = datadrivingimu1(:,27:28);
ang_velZ = datadrivingimu1(:,17);
quaternion = datadrivingimu1(:,10:13);
mag_driving = table2array(mag_driving);
ang_velZ = table2array(ang_velZ);
quaternion = table2array(quaternion);
len = (length(mag_driving)-1);
t = [(0:len)/40].';


% Calculating yaw from raw magnetometer data
yaw = atan2(-mag_driving(:,2),mag_driving(:,1));

figure
% Plotting the raw magnetometer yaw
plot(t,unwrap(yaw)-1.58,'r')

hold on;

% Using the magnetometer calibration data to calibrate the magnetometer
% data
translation = mag_driving(:,:).'-[mt(1,1)*ones(1,size(mag_driving,1));mt(2,1)*ones(1,size(mag_driving,1))];
mag_driving_cal = ms*mrot*translation;
mag_driving_cal = mag_driving_cal(:,:).';

% Calculating yaw from calibrated magnetometer data
yaw_cal= atan2(-mag_driving_cal(:,2),mag_driving_cal(:,1));


% Unwraping the magnetometer yaw data and removing the bias
yaw_cal_unwrap = unwrap(yaw_cal)-1.58;

% Plotting the calibrated magnetometer yaw
plot(t,yaw_cal_unwrap,'b')
title("Magnetometer yaw")
xlabel("Time (sec)")
ylabel("Yaw magnetometer (rad)")
legend("Raw magnetometer yaw","Calibrated magnetometer yaw")
hold off;

%% Plotting yaw from gyro and calibrated magnetometer yaw

% Integrating angular velocity to find the yaw and removing the bias
yaw_gyro = cumtrapz(t,ang_velZ(:,:))-1.62;
figure
hold on
plot(t,yaw_gyro,'r')
plot(t,yaw_cal_unwrap,'b')
title("Magnetometer Yaw and Gyro integrated yaw")
xlabel("time (sec)")
ylabel("Yaw (rad)")
legend("Gyro Integrated Yaw","Magnetometer Yaw")
hold off

%%  Passing the magnetometer yaw from low pass filter and gyro yaw from high pass filter

% Passing the magnetometer data through lowpass filter
lowpass_yaw = lowpass(yaw_cal_unwrap,0.5,40);

% Passing the gyroscope data through highpass filter
highpass_yaw = highpass(yaw_gyro,0.5,40);

% Computing the complimentary filter by adding the lowpass filter and
% highpass filter output
comp = lowpass_yaw + highpass_yaw;

figure
hold on
plot(t,lowpass_yaw,'b')
plot(t,highpass_yaw,'g')
title('Filtered Yaw data')
xlabel("time (sec)")
ylabel("yaw (rad)")
legend("Lowpass filter","Highpass filter")
hold off

%% Calculating the yaw angle from the IMU

% Converting the quaternion to euler angle
quat_wxyz = [quaternion(:,4) quaternion(:,1:3)];
eulerAngle = quat2eul(quat_wxyz);

figure
hold on
plot(t,unwrap(eulerAngle(:,1)),'k')
plot(t,comp,'r')
title("Filtered yaw and IMU Yaw")
xlabel("time (sec)")
ylabel("yaw (rad)")
legend('imu','comp')
hold off

save('filtered_comp_yaw.mat',"comp")
