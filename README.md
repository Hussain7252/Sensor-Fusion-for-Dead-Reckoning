The analysis is done in Matlab.

In order to run the program, make sure that all the mat files and matlab programs are in the same folder.

Following program can be run to find the respective output

# For finding the heading
imucircle1.mat       #Matlab Data file
imudriving1.mat      #Matlab Data file
heading_yaw.m        #Matlab script for analysis

Note: fit_ellipse.m contains a function which is called in heading_yaw.m, so do not run the fit_ellipse.m file.

# For finding the forward velocity
imudriving1.mat        #Matlab Data file
gpsdriving1.mat        #Matlab Data file
forward_velocity.m     #Matlab script for analysis

# For Dead Reckoning
imudriving1.mat        #Matlab Data file
gpsdriving1.mat        #Matlab Data file
dead_reckoning.m       #Matlab script for analysis

# For estimating the Xc
imudriving1.mat        #Matlab Data file
Xc_centeroffset.m         #Matlab script for analysis


