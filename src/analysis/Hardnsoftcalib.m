load('imucircle.mat')
Mag_X_circle=imucircle(:,27);
Mag_Y_circle=imucircle(:,28);

Magx_c_a=table2array(Mag_X_circle);
Magy_c_a=table2array(Mag_Y_circle);
Magx_c_a=Magx_c_a(3242:5528);
Magy_c_a=Magy_c_a(3242:5528);
%Hard Iron Calibration
xcmax=max(Magx_c_a);
xcmin=min(Magx_c_a);
ycmax=max(Magy_c_a);
ycmin=min(Magy_c_a);

xoffset=(xcmax+xcmin)/2;
yoffset=(ycmax+ycmin)/2;

h_calib_x=Magx_c_a - xoffset;
h_calib_y=Magy_c_a - yoffset;

%Plotting After Hard Iron Calibration
figure
scatter(h_calib_x,h_calib_y)
hold on
grid on
scatter(Magx_c_a,Magy_c_a)
hold on
scatter(0,0,'x')
hold on 
scatter((max(h_calib_x)+min(h_calib_x))/2,(max(h_calib_y)+min(h_calib_y))/2,'d')
legend('hard_calibrated','hard_uncalibrated','origin(0,0)','calibrated_data origin')
title('Hard_Calibrated & Hard_uncalibrated')
xlabel("Magnetometer x")
ylabel("Magnetometer y")

%Soft Iron Calibration
ellipse=fit_ellipse(h_calib_x,h_calib_y);

radians=ellipse.phi;
angle=radians .* (180/pi);
disp(angle)
Rotation_Matrix=[cos(radians) sin(radians);-1.*sin(radians) cos(radians)];
vector=[h_calib_x h_calib_y];
Rotated_Values=vector*Rotation_Matrix;
Calib_X=Rotated_Values(:,1);
Calib_Y=Rotated_Values(:,2);

major_axis=ellipse.long_axis;
minor_axis=ellipse.short_axis;
scale=major_axis ./ minor_axis;
s_calib_x= Calib_X./ scale;

%plotting
figure
scatter(Magx_c_a,Magy_c_a)
hold on
grid on
scatter(s_calib_x,Calib_Y)
hold on
scatter(h_calib_x,h_calib_y)
hold on
scatter(0,0,'x')
hold on 
scatter((max(s_calib_x)+min(s_calib_x))/2,(max(Calib_Y)+min(Calib_Y))/2,'d')
legend('uncalibrated','soft_calibrated','hard_calibrated','origin(0,0)','soft_calibrated_data origin')
title('Soft_Calibrated & Hard_Calibrated & Uncalibrated')
xlabel("Magnetometer x")
ylabel("Magnetometer y")

%Displaying the value of Soft and Hard Calibrations
disp("Major axis and Minor Axis Length")
disp(major_axis)
disp(minor_axis)
disp("Soft Calibrated Value:-")
disp(scale)
disp('Hard Calibrated Values X and Y')
disp(xoffset)
disp(yoffset)
disp('Angle Rotation in Radians and Degree')
disp(radians)
disp(angle)