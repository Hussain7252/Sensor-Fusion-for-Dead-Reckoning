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
figure()
plot(Time,linear_acc_x)
title('Time vs Linear_acc_uncorrected')

%GPS VELOCITY CALCULATION
figure(1)
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
plot(time_gps,gps_velocity)
xlabel('Time')
ylabel('Velocity(m/sec)')
title('GPS Velocity')
%Calculating Jerk and Correcting Based on it
figure()
try4data=linear_acc_x;
jerk=diff(try4data);
t=transpose([1:length(jerk)]);
t_try4=transpose([1:length(try4data)]);
disp(jerk)
plot(t,jerk)
hold on
plot(t_try4,try4data)
title('JERK')
%Getting Stop Time Intrevals
th=0.04;
conse=50;
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
%{
mea=[];
start1=[];
if start(1) ~= 1
    start1(end+1,1)=1;
    mea(end+1,1)=mean(try4data(start(1):ed(1)));
end
for i=1:length(ed)
    mea(end+1,1)=mean(try4data(start(i):ed(i)));
end

for i=1:length(start)
    start1(end+1,1)=start(i);
end
start1(end+1,1)=length(try4data);

for i=1:length(mea)
    try4data(start1(i):(start1(i+1)))=(try4data(start1(i):(start1(i+1)))-mea(i));
end
%}
figure()
plot(Time,cumtrapz(Time,puredata_lst))
hold on
plot(time_gps,gps_velocity)
legend('ACC_corr_velocity','GPS_VELOCITY')
title("GPS VS Acc. Velocity")
xlabel("Time(sec)")
ylabel("Velocity (m/s)")

figure()
plot(Time,cumtrapz(Time,linear_acc_x))
hold on
plot(time_gps,gps_velocity)
legend('ACC_uncorr_velocity','GPS_VELOCITY')
title("GPS VS UNCORR. Acc. Velocity")
xlabel("Time(sec)")
ylabel("Velocity (m/s)")

figure()
plot(Time,puredata_lst)
hold on
plot(Time,linear_acc_x)
legend('Corr acc.','uncorr acc.')
title('Corrected vs Uncorrected acc.')
xlabel('Time(sec)')
ylabel('Acc. (m/s^2)')

figure()
plot(Time,cumtrapz(Time,linear_acc_x))
hold on
plot(Time,cumtrapz(Time,puredata_lst))
legend('Uncorrected Velocity','Corrected Velocity')
title('Velocity from Acc.')
xlabel('Time')
ylabel('Velocity (m/s)')
%Integrating the forward Acceleration To get forward Velocity
%figure
%forward_vel=cumtrapz(Time,linear_acc_x);
%plot(Time,forward_vel)
%title('Forward Velocity gyro uncorrected')

%calculate velocity from gps and plot gpsvelocity and acceleration
%plot(Time,linear_acc_x)
%{
%Correcting Acceleration and removing bias due to Z
%This bias in accelerometer_x is added due to pitch 
%So I will be calculating the gravity component in X direction and
%subtracting it from Linear acceleration in X direction for each and every
%point
figure
pitch=table2array(imudriving(:,32));
linear_acc_z=table2array(imudriving(:,21));
for j=1:(length(pitch))
   comp_in_x(j)=linear_acc_z(j)* sin(pitch(j)*(pi./180));
end
disp(comp_in_x)
comp_in_x=transpose(comp_in_x);
linear_accx_corr=linear_acc_x + comp_in_x;
plot(Time,linear_accx_corr)
title('Removing Pitch Effect')
%v=0.00001:0.000010:0.0001;

figure()
corrected_velocity=cumtrapz(Time,linear_accx_corr);
plot(Time,corrected_velocity)
hold on
%for k=1:length(v)
%    y = highpass(corrected_velocity,v(k),40);
%    error=abs(corrected_velocity-y);
%    total=sum(error);
%    ec(k)=total;
%end
%[M,I]=min(ec);
y = highpass(corrected_velocity,0.0001,40);
plot(Time,y)
hold on 
plot(time_gps,gps_velocity)
legend('Pitch_corr_vel','HighPass_velo','gps_velocity')


%This Process Doesnot Work as effect of acceleration due to gravity is not
%zeroing out

%So The High Pass Filter method also fails to correctly account for the
%errors
%start=0;
%store=[];
%for h=1:30:length(pitch)-30
%    mag=abs(pitch(h+30)-pitch(h));
%    if mag>1
%        stop=h;
%        store(end+1)=h;
%        start=h+1;
%    end
%end

%Divide the dataset into parts and take mean
figure()
L=1:29056;
M=mean(linear_accx_corr);
Meanacc=linear_accx_corr-M;

Meanvel=cumtrapz(Time,Meanacc);
plot(Time,Meanvel)
hold on
grid on
plot(time_gps,gps_velocity)
hold on
plot(Time,y)
%}