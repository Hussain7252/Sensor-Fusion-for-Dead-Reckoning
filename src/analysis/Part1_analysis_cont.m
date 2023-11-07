load("imudriving.mat");
%Data Extraction
yaw=imudriving(:,31);
yaw=table2array(yaw);
yaw=(yaw.*(pi/180))-2.2928;
yaw=unwrap(yaw);
yaw=yaw.*(180/pi);
time_nsec=imudriving(:,4);
time_sec=imudriving(:,3);
time_nsec=table2array(time_nsec);
time_sec=table2array(time_sec);
time_conv=time_nsec .* 10^-9;
tt_sec=time_sec + time_conv;
mintime= min(tt_sec);
Time=tt_sec - mintime;
Mag_X=imudriving(:,27);
Mag_Y=imudriving(:,28);
Mag_X_arr=table2array(Mag_X);
Mag_Y_arr=table2array(Mag_Y);

%Hardiron Calibration
x_offset=-0.0550; 
y_offset=0.0315;
h_calib_X=Mag_X_arr-x_offset;
h_calib_Y=Mag_Y_arr-y_offset;

%Soft Iron Calibration
scale=1.3544;
radians=-0.0144;
Rotation_Matrix=[cos(radians) sin(radians);-1.*sin(radians) cos(radians)];
vector=[h_calib_X h_calib_Y];
Rotated_Values=vector*Rotation_Matrix;
Calib_X=Rotated_Values(:,1);
Calib_Y=Rotated_Values(:,2);
s_calib_X=Calib_X./scale;

%Plotting Yaw(degree) With Time
figure
plot(Time,yaw)
grid on
xlabel('Time')
ylabel('Yaw in degree')

%Converting Magnetometer X and Y Value to Yaw(radian)
%figure
rad_direct=atan2(-Calib_Y,s_calib_X)-3.0728;
figure()
plot(Time,rad_direct)
grid on
xlabel('Time')
ylabel('calibrated_Mag_yaw')

%Making corrections to remove noise due to train
correction=rad_direct(20000:26500);
y = movmean(correction,50);
rad_direct(20000:26500)=y;
figure
plot(Time,rad_direct)
grid on
xlabel('Time')
ylabel('Noise_free_Magnetometer_Yaw')
title("Train Noise Remove")

%UNWRAP Calibrated Mag YAW
yawcalib_radians=unwrap(rad_direct);
yawcalib_degree=yawcalib_radians .* (180./pi);
figure
plot(Time,yawcalib_degree)
grid on
xlabel('Time')
ylabel('Calibrated_Yaw_degrees')
%Hence With This Yaw is Calculated for Calibrated Magnetometer


%Raw Data Yaw Calculation
%yaw_radian_raw=unwrap(atan2(-Mag_Y_arr,Mag_X_arr));
yaw_radian_raw=atan2(-Mag_Y_arr,Mag_X_arr);

correction=yaw_radian_raw(20000:26500);
y = movmean(correction,50);
yaw_radian_raw(20000:26500)=y;

yaw_radian_raw=unwrap(yaw_radian_raw);
yaw_angle_raw=yaw_radian_raw .* (180/pi);
figure
plot(Time,yaw_angle_raw)
hold on
grid on
plot(Time,yawcalib_degree)
legend('Raw_magnetometer','Calibrated_Magnetometer')
title('YAW IN DEGREE')
xlabel("Time")
ylabel('Yaw degree')


%Gyro Conv
figure(10)
ang_vel_z=imudriving(:,17);
ang_vel_z=table2array(ang_vel_z);
rad_gyro=cumtrapz(Time,ang_vel_z);
ang_gyro=rad_gyro .* (180/pi);
plot(Time,ang_gyro)
hold on
plot(Time,yawcalib_degree)
%hold on
%plot(Time,yaw)
legend('gyro_yaw','mag_yaw')
xlabel('Time(sec)')
ylabel('Yaw(degree)')
%hold off

%{
%Forier Transform
figure()
L = length(yawcalib_degree);
n = 2^nextpow2(L);
Fs=40;
fft1 = fft(yawcalib_degree,n);
f = Fs*(0:(n/2))/n;
P = abs(fft1/n).^2;

plot(f,P(1:n/2+1)) 
title("Gaussian Pulse in Frequency Domain")
xlabel("f (Hz)")
ylabel("|P(f)|^2")
%}
%Passing Magnetometer Yaw to low pass filter
%lowpass_mag_yaw = lowpass(yawcalib_degree,0.8,40);
lowpass_mag_yaw = lowpass(yawcalib_degree,0.0002);
figure(27)
title('Mag_yaw from LPF ')
plot(Time,lowpass_mag_yaw)
xlabel('Time')
ylabel('YAW in degree')
%hold on
%grid on
%plot(Time,yaw)

%passing gyro to high pass filter 
highpass_gyro_yaw=highpass(ang_gyro,0.00002,40);
highpass_gyro_yaw=highpass_gyro_yaw-(highpass_gyro_yaw(1)-0);
figure(16)
title('gyro_yaw with HPF and without')
plot(Time,highpass_gyro_yaw)
hold on
grid on
plot(Time,yaw)

%Complementary filter
a=0.6;
b=0.000001;
c=1-a;
d=1;
Comp_mag=lowpass_mag_yaw;
Comp_mag(1:20000)=a.*lowpass_mag_yaw(1:20000);
Comp_mag(20000:length(Comp_mag))=b.*lowpass_mag_yaw(20000:length(Comp_mag));
Comp_gyro=highpass_gyro_yaw;
Comp_gyro(1:20000)=c.*highpass_gyro_yaw(1:20000);
Comp_gyro(20000:length(Comp_gyro))=d.*Comp_gyro(20000:length(Comp_gyro));
Comp_yaw=Comp_mag+Comp_gyro;

Comp_Corr = movmean(Comp_yaw(19240:20800),50);
Comp_yaw(19240:20800)=Comp_Corr;
figure()
plot(Time,Comp_yaw)
hold on
plot(Time,lowpass_mag_yaw)
hold on
plot(Time,highpass_gyro_yaw)
legend('YAW CF','YAW LPF Magnetometer','YAW HPF Gyro')
title('YAW from CF, LPF MAGNETOMETER, HPF GYRO')
xlabel('Time(sec)')
ylabel('Yaw(degree)')
figure()
plot(Time,yaw)
hold on
plot(Time,Comp_yaw)
legend('YAW IMU','YAW CF')
title('YAW from IMU AND CF')
xlabel('Time(sec)')
ylabel('Yaw(degree)')
