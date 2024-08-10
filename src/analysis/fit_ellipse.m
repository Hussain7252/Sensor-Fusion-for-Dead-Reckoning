function ellipse_t = fit_ellipse( x,y )
x1 = x(:);
y1 = y(:);
% initialize
orientation_tolerance = 1e-3;
% empty warning stack
lastwarn( '' );
% prepare vectors, must be column vectors
x = x(:);
y = y(:);
% remove bias of the ellipse - to make matrix inversion more accurate. (will be added later on).
mean_x = mean(x);
mean_y = mean(y);
x = x-mean_x;
y = y-mean_y;
% the estimation for the conic equation of the ellipse
X = [x.^2, x.*y, y.^2, x, y ];
a = sum(X)/(X'*X);

% extract parameters from the conic equation
[a,b,c,d,e] = deal( a(1),a(2),a(3),a(4),a(5) );
% remove the orientation from the ellipse
if ( min(abs(b/a),abs(b/c)) > orientation_tolerance )
    
    orientation_rad = 1/2 * atan( b/(c-a) );
    cos_phi = cos( orientation_rad );
    sin_phi = sin( orientation_rad );
    [a,b,c,d,e] = deal(...
        a*cos_phi^2 - b*cos_phi*sin_phi + c*sin_phi^2,...
        0,...
        a*sin_phi^2 + b*cos_phi*sin_phi + c*cos_phi^2,...
        d*cos_phi - e*sin_phi,...
        d*sin_phi + e*cos_phi );
    [mean_x,mean_y] = deal( ...
        cos_phi*mean_x - sin_phi*mean_y,...
        sin_phi*mean_x + cos_phi*mean_y );
else
    orientation_rad = 0;
    cos_phi = cos( orientation_rad );
    sin_phi = sin( orientation_rad );
end
    
    % make sure coefficients are positive as required
    if (a<0), [a,c,d,e] = deal( -a,-c,-d,-e ); end
    
    % final ellipse parameters
    X0          = mean_x - d/2/a;
    Y0          = mean_y - e/2/c;
    F           = 1 + (d^2)/(4*a) + (e^2)/(4*c);
    [a,b]       = deal( sqrt( F/a ),sqrt( F/c ) );    
    long_axis   = 2*max(a,b);
    short_axis  = 2*min(a,b);
    % rotate the axes backwards to find the center point of the original TILTED ellipse
    R           = [ cos_phi sin_phi; -sin_phi cos_phi ];
    P_in        = R * [X0;Y0];
    X0_in       = P_in(1);
    Y0_in       = P_in(2);
    
    % pack ellipse into a structure
    ellipse_t = struct( ...
        'a',a,...
        'b',b,...
        'phi',orientation_rad,...
        'X0',X0,...
        'Y0',Y0,...
        'X0_in',X0_in,...
        'Y0_in',Y0_in,...
        'long_axis',long_axis,...
        'short_axis',short_axis,...
        'status','' );

% rotation matrix to rotate the axes with respect to an angle phi
R = [ cos_phi sin_phi; -sin_phi cos_phi ];
    
% the axes
ver_line        = [ [X0 X0]; Y0+b*[-1 1] ];
horz_line       = [ X0+a*[-1 1]; [Y0 Y0] ];
new_ver_line    = R*ver_line;
new_horz_line   = R*horz_line;
    
% the ellipse
theta_r         = linspace(0,2*pi);
ellipse_x_r     = X0 + a*cos( theta_r );
ellipse_y_r     = Y0 + b*sin( theta_r );
rotated_ellipse = R * [ellipse_x_r;ellipse_y_r];
    
% draw
axis equal
hold on
plot( new_ver_line(1,:),new_ver_line(2,:),'r-');
plot( new_horz_line(1,:),new_horz_line(2,:),'g-');
plot( rotated_ellipse(1,:),rotated_ellipse(2,:),'k',LineWidth=3);
plot(x1,y1,'.')
axis equal
xlabel("magnetic x (gauss)")
ylabel("magnetic y (gauss)")
legend("minor axis","major axis","bestFitEllipse","actual data")

end