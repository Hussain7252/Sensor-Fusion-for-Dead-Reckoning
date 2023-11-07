%Data Extraction and Time calculation
load('imudriving.mat')
load('gpsdriving.mat')
linear_acc_x=imudriving(:,19);
linear_acc_x=table2array(linear_acc_x);
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
yaw=imudriving(:,31);
yaw=table2array(yaw);
yaw=(yaw.*(pi/180))-2.2928;
yaw=unwrap(yaw);
yaw=yaw.*(180/pi);
ang_vel_z=imudriving(:,17);
ang_vel_z=table2array(ang_vel_z);

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

%Converting Magnetometer X and Y Value to Yaw(radian)
rad_direct=atan2(-Calib_Y,s_calib_X)-3.0728;

%Making corrections to remove noise due to train
correction=rad_direct(20000:26500);
y = movmean(correction,50);
rad_direct(20000:26500)=y;

%UNWRAP Calibrated Mag YAW
yawcalib_radians=unwrap(rad_direct);
yawcalib_degree=yawcalib_radians .* (180./pi);

%Gyro Conv
rad_gyro=cumtrapz(Time,ang_vel_z);
ang_gyro=rad_gyro .* (180/pi);
%Low Pass Filter
lowpass_mag_yaw = lowpass(yawcalib_degree,0.0002);

%high Pass Filter
highpass_gyro_yaw=highpass(ang_gyro,0.00002,40);
highpass_gyro_yaw=highpass_gyro_yaw-(highpass_gyro_yaw(1)-0);

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
plot(Time,yaw)
hold on
plot(Time,Comp_yaw)
legend('YAW IMU','YAW CF')
title('YAW from IMU AND CF')
xlabel('Time(sec)')
ylabel('Yaw(degree)')
%GPS VELOCITY CALCULATION
Easting=table2array(gpsdriving(:,9));
Northing=table2array(gpsdriving(:,10));

for i=1:(length(Easting)-1)
    e0=Easting(i);
    n0=Northing(i);
    e1=Easting(i+1);
    n1=Northing(i+1);
    D=sqrt((e1-e0).^2 + (n1-n0).^2);
    gps_velocity(i)=D;
end
gps_velocity=transpose(gps_velocity);
time_gps=transpose([1:length(gps_velocity)]);

%Acceleration and Velocity Correction
try4data=linear_acc_x;
jerk=diff(try4data);
t=transpose([1:length(jerk)]);
t_try4=transpose([1:length(try4data)]);
th=0.05;
conse=120;
stop_periods=[];
counter=0;
for i=1:length(jerk)
    ch=counter;
    if abs(jerk(i))<th
        counter=counter+1;
    else
        counter=0;
    end
    if ch>conse && counter==0
        stop_periods(end+1,1)=i-ch;
        stop_periods(end+1,1)=i-1;
    end
end
puredata_lst=ones(length(linear_acc_x),1);
for i=2:length(stop_periods)
    mean_noise=mean(linear_acc_x(stop_periods(i-1):stop_periods(i)));
    if i==2
        st=1;
    else
        st=stop_periods(i-1);
    end
    if i<length(stop_periods)
        edd=stop_periods(i+1);
    else
        edd=length(linear_acc_x);
    end
    for j=st:edd
        puredata_lst(j)=linear_acc_x(j)-mean_noise;
    end
end
n=length(stop_periods);
stl=stop_periods(n-1);
enl=length(puredata_lst);
puredata_lst(stl:enl)=puredata_lst(stl:enl)-mean(puredata_lst(stl:enl));
figure()
plot(Time,cumtrapz(Time,puredata_lst))
hold on
plot(time_gps,gps_velocity)
legend('ACC_corr_velocity','GPS_VELOCITY')
title("GPS VS Acc. Velocity")
xlabel("Time(sec)")
ylabel("Velocity (m/s)")

%PART C DR
velocity_acc=cumtrapz(Time,puredata_lst);

%Integrating Forward Velocities to obtain displacement and PLOTTING
figure()
plot(Time,cumtrapz(Time,velocity_acc))
hold on
plot(time_gps,cumtrapz(time_gps,gps_velocity))
legend('Dis. Acc','Dis. gps')
title('DISTANCE')
xlabel('Time')
ylabel('Distance (m)')

%Path Followed by GPS sensor
figure()
plot(Easting,Northing)
xlabel('Easting(m)')
ylabel('Northing (m)')
title('GPS estimate travel')
easting=Easting-Easting(1);
northing=Northing-Northing(1);
figure()
plot(easting,northing)

%Calculate Trajectory Using Accelerometer and complimentary filter
Comp_radian=Comp_yaw.*(pi/180);
Comp_radian=Comp_radian+1+0.174;
Xcomp=cos(Comp_radian);
Ycomp=-sin(Comp_radian);
Xaxis=velocity_acc.*Xcomp;
Yaxis=velocity_acc.*Ycomp;
Xaxisdisp=cumtrapz(Time,Xaxis);
Yaxisdisp=cumtrapz(Time,Yaxis);
figure()
plot(Xaxisdisp,Yaxisdisp)
hold on
plot(easting,northing)
legend('IMU trajectory',"GPS trajectory")
title('Path Travelled')
xlabel('East(meters)')
ylabel('North(meters)')

%Comparing velocity*yaw_rate with linear acceleration Y
cb=velocity_acc.*ang_vel_z;
linear_acc_y=table2array((imudriving(:,20)));

figure()
plot(Time,cb)
hold on
plot(Time,linear_acc_y)
legend('X(dot)*W','linear_acc_y')
title('X(dot)*W vs Uncorrected Linear acc. Y')
xlabel('Time(sec)')
ylabel('Acc_y')

jerk_y=diff(linear_acc_y);
th=0.05;
conse=120;
stop_periods=[];
counter=0;
for i=1:length(jerk_y)
    ch=counter;
    if abs(jerk_y(i))<th
        counter=counter+1;
    else
        counter=0;
    end
    if ch>conse && counter==0
        stop_periods(end+1,1)=i-ch;
        stop_periods(end+1,1)=i-1;
    end
end
puredata_lst_y=ones(length(linear_acc_y),1);
for i=2:length(stop_periods)
    mean_noise=mean(linear_acc_y(stop_periods(i-1):stop_periods(i)));
    if i==2
        st=1;
    else
        st=stop_periods(i-1);
    end
    if i<length(stop_periods)
        edd=stop_periods(i+1);
    else
        edd=length(linear_acc_y);
    end
    for j=st:edd
        puredata_lst_y(j)=linear_acc_y(j)-mean_noise;
    end
end
n=length(stop_periods);
stl=stop_periods(n-1);
enl=length(puredata_lst_y);
puredata_lst_y(stl:enl)=puredata_lst_y(stl:enl)-mean(puredata_lst_y(stl:enl));
figure()
plot(Time,cb)
hold on
plot(Time,puredata_lst_y)
legend('X(dot)W','Corr_linear_acc_y')
title('X(dot)W and Corrected acc. Y')


xlabel('Time')
ylabel("Acc")
