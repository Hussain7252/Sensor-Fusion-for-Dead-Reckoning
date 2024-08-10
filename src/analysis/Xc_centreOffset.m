%% Estimating the Center offset of IMU from COM

% Loading the imu data taken in circle
load('imudriving1.mat');

% Extracting the magnetic field data
acceleration = datadrivingimu1(:,19:21);
ang_velZ = datadrivingimu1(:,17);
acceleration= table2array(acceleration);
w = table2array(ang_velZ);
ax = acceleration(:,1);
ay = acceleration(:,2);
l = length(ax);
t = (0:l-1)/40;
% t = t.';
% dt = diff(t);
dt = 0.025;

% Solving the following equation to find out the Xc
%((w')^2-w^4-(w*w'))Xc = w^2*acc_x-w*acc_y'+w'*y''

omegaSq_ax = (w.^2).*ax;

aydot = diff(ay)/dt;
aydot(l) = aydot(l-1);
omega_aydot = w.*aydot;

omegadot = diff(w)/dt;
omegadot(l) = omegadot(l-1);
omegadot_ay = omegadot.*ay;

B = omegaSq_ax-omega_aydot+omegadot_ay;

omegadotSq = omegadot.^2;

omegafour = w.^4;

omegadotdot = diff(omegadot)/dt;
omegadotdot(l) = omegadotdot(l-1);
omega_omegadotdot = w.*omegadotdot;

A = omegadotSq-omegafour-omega_omegadotdot;
Xc = linsolve(A,B);

fprintf("\nEstimated value of Xc=%f m\n",Xc)