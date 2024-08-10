%% Parsing the IMU and GPS data

% Loading the imu data taken in circle
load('imudriving1.mat');
load('gpsdriving1.mat');
load('vel_forward.mat');
load('filtered_comp_yaw.mat');

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
time1 = time-min(time);

%% Calculating the displacement from IMU forward velocity and GPS data

% Integrating corrected IMU forward velocity to get forward distance
dist_forward = cumtrapz(t,corr_vel);

% Finding the displacement from GPS data
gps_dist = [];
gps_dist(1,1) = 0;
for i= 2:length(east)
    dy = east(i)-east(i-1);
    dx = north(i)-north(i-1);
    gps_dist(i) = gps_dist(i-1) + ((dx^2+dy^2)^0.5);
end


% Plotting the displacement graph
figure
hold on
plot(t,dist_forward,'b')
plot(time1,gps_dist,'r')
title("Displacement graph of IMU and GPS")
xlabel("time (sec)")
ylabel("Distance (m)")
legend("IMU displacement","GPS displacement")
hold off


% Finding the coriolis acceleration and comparing it with acceleration in y
% direction of IMU
coriolis_acc = ang_velZ.*corr_vel;
figure
hold on
plot(t,acc_y,'b')
plot(t,coriolis_acc,'r',LineWidth=1.5)
title("IMU Linear acceleration in Y direction and wxdot plot")
xlabel("Time (sec)")
ylabel("Acceleration (m/s^2)")
legend("IMU Y acceleration","\omega dot{X}")
hold off

%% Estimating the Dead reckoning 

% Adjusting the heading to orient as per the GPS 
comp = comp + 3.2;

% Finding the velocity in east and north direction
vel_east= -1*corr_vel.*cos(comp(1:end));
vel_north= corr_vel.*sin(comp(1:end));

% Calculating the position estimate in east and north direction
pos_east = cumtrapz(t(1:end),vel_east);
pos_north = cumtrapz(t(1:end),vel_north);

% Bringing the start point at the origin
east_true = east - east(1);
north_true = north - north(1);

% Plottting the Dead Reckoning eastimate and GPS trajectory
figure
hold on
plot(east_true,north_true,'ob-',LineWidth=2)
plot(pos_east,pos_north,'or-',LineWidth=2)
plot(pos_east(1),pos_north(1),'og',LineWidth=2,MarkerSize=15)
plot(pos_east(end),pos_north(end),'*k',LineWidth=2,MarkerSize=15)
title("Dead reckoning estimate and GPS trajectory")
xlabel("East (m)")
ylabel("North (m)")
legend("GPS Trajectory","Dead Reckoning estimate","Start Point of DR", "End point of DR")
hold off




