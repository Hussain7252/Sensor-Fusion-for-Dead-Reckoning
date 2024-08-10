%% Calculating Forward velocity

% Loading the imu data taken in circle
load('imudriving1.mat');
load('gpsdriving1.mat');

% Extracting the magnetic field data
acceleration = datadrivingimu1(:,19:21);
ang_velZ = datadrivingimu1(:,17);
acceleration= table2array(acceleration);
ang_velZ = table2array(ang_velZ);
acc_x = acceleration(:,1);
acc_y = acceleration(:,2);
len = length(acc_x);
t = (0:len-1)/40;

% Extracting the gps data
east = datadrivinggps1(:,9);
north = datadrivinggps1(:,10);
time = datadrivinggps1(:,3);
east = table2array(east);
north = table2array(north);
time = table2array(time);

%% Calculating velocity from Accelerometer and GPS

% Plotting the Acceleration data
figure
plot(t,acc_x)
title("Forward acceleration")
xlabel("Time (sec)")
ylabel("Acceleration (m/s^2)")

% Integrating acceleration in x to find forward velocity
forward_vel_uncorr = cumtrapz(t,acc_x);

% Finding velocity from GPS
time1 = time-min(time);
gps_vel = [];
gps_vel(1,1) = 0;
for i= 2:length(east)
    dy = east(i)-east(i-1);
    dx = north(i)-north(i-1);
    dt = time(i)-time(i-1);
    gps_vel(i) = ((dx^2+dy^2)^0.5)/dt;
end
gps_vel = gps_vel.';


% Plotting uncorrected forward velocity from accelerometer and velocity
% from GPS
figure
hold on
plot(t,forward_vel_uncorr,'b')
plot(time1,gps_vel,'r')
title("IMU Uncorrected velocity and GPS velocity")
xlabel("time (sec)")
ylabel("velocity (m/s)")
legend("IMU forward velocity","GPS velocity")
hold off

jerk = diff(acc_x)/0.025;
figure
plot((0:len-2)/40,jerk)
title("Forward Jerk")
xlabel("Time (sec)")
ylabel("Jerk (m/s^3)")

%% Removal of Acceleration bias and calculating corrected forward velocity

% Finding the stop time from acceleration and Jerk plot
stopTime = [0,1666,3835,4953,8240,9071];

% Finding the offset for each stop interval
accel_offset = {};
ts = {};
offset_array = zeros(length(stopTime),1);
for i=1:length(stopTime)
    if i<length(stopTime)
        accel_offset(i,:) = {acc_x(stopTime(i)+1:stopTime(i+1))};
        ts(i,:) = {t(stopTime(i)+1:stopTime(i+1))};
    else
        accel_offset(i,:) = {acc_x(stopTime(i)+1:length(acc_x))};
        ts(i,:) = {t(stopTime(i)+1:length(acc_x))};
    end
    bias = mean(accel_offset{i,:})+0.002;
    offset_array(i)= bias;
end

% Integrating over each interval with offset to find the corrected velocity
corr_vel = [];
ts_final = [];
for i=1:length(accel_offset)
    temp = cumtrapz(ts{i,:},accel_offset{i,:}-offset_array(i));
    corr_vel = [corr_vel;temp];
    ts_final = [ts_final, ts{i,:}];
end
negIndex = corr_vel < 0;
corr_vel(negIndex) = 0;

% Plotting the Corrected IMU velocity and GPS velocity
figure
hold on
plot(t,corr_vel,'b')
plot(time1,gps_vel,'r')
title("IMU corrected velocity and GPS velocity")
xlabel("time (sec)")
ylabel("velocity (m/s)")
legend("IMU forward velocity","GPS velocity")
hold off


save('vel_forward.mat','corr_vel','gps_vel')
