%--------------------------------------------------------------------
%
% File: stereo_specs.m
% Author: Jeffrey Byrne (jebyrne@gmail.com)
%
% Description:  Script that will compute stereo parameters for a 
% given fixed specification
%
%--------------------------------------------------------------------

% Constants 
%--------------------------------------------------------------------

% Baseline (m)
B = 2;

% Pixel size 
P = 0.000005;
D_H = 1936;
D_V = 1086;

% Lens focal length (m)
f = 0.016;

% Circle of confusion diameter (m)
c = 1.5*P;

% F-stop number (focal length/aperture diameter)
% Standard f-numbers are powers of sqrt(2)
%a = sqrt(2)^6;  % f/8
a = sqrt(2)^7;    % f/11
%a = sqrt(2)^8;  % f/16


% Focusing distance: distance at which the camera is focused (m)
r_f = 2000;  % So that far focus is infinity
%r_f = ((f.^2)/(a*c)) + f;   % So that we can focus on close objects

% Detection certainty
A = 1;

% Number of pixels on target for minimum detection
U = 1;

% Minimum disparity search
d_min =  2;

% Maximum disparity search
d_max =  d_min+128;

% Stereo subpixel interpolation
p_i = 0.25;


% Vehicle parameters
%--------------------------------------------------------------------
wingspan = 2.4;
body = 1.5;


% Derived variables
%--------------------------------------------------------------------
% Camera angle of view (radians)
% -Verified with camera calibration
haov = 2*atan((D_H*P)/(2*f));
vaov = 2*atan((D_V*P)/(2*f));

% Range from the stereo origin to the initial stereo (camera) overlap
% -Verified with camera calibration
r_co = (B/2)*tan(pi/2 - haov/2);
  
% Maximum range from the stereo origin to a minimum disparity distance
r_max = (B*f)/(d_min*P);
  
% Stereo field of view (Phi) (radians)
p = 2*atan(B/(2*r_co));
  
% Minimum range to maximum disparity
r_min = (B*f)/(d_max*P);

% Minimum resolvable obstacle dimension at range r
o_min = (U*P)/(f); % times r

% Obstacle dimension that projects to U pixels at r_max (max resolvable range)
o_r_max = (r_max*U*P)/(f);

% Obstacle dimension that projects to U pixels at r_min (minimum resolvable range)
o_r_min = (r_min*U*P)/(f);

% Range resolution at maximum resolvable range
rr_max = ((r_max.^2)/(B*f))*(P*p_i);

% Range resolution at minimum resolvable range
rr_min = ((r_min.^2)/(B*f))*(P*p_i);

% Range at which range resolution is 1cm
r_rr1cm = sqrt((0.01*B*f)/(P*p_i));

% Hyperfocal distance (m)
h = ((f.^2)/(a*c)) + f;

% Near focus distance (m)
% Note:  r_nf ~= (r_f * h) / ( h + r_f)
r_nf = (r_f*(h-f))/(h+r_f-2*f);

% Far focus distance (m)
% Note: r_ff ~= (r_f * h) / ( h - d ); r_f=h --> r_ff=inf
r_ff = (r_f*(h-f))/(h-(r_f-f));
if (r_ff < 0)
    r_ff = inf;
end

% Maximum wing baseline for nose visibility
B_wing = 2*((body/2)-0.1) / (tan((pi/2) - (haov/2)));

% Maximum body baseline for wing visibility
B_body = 2*((wingspan/2)-0.1) / (tan((pi/2) - (haov/2)));

% Wingcam to nose distance
r_w2n = sqrt((body/2)^2 + (B_wing/2)^2);

% Minimum dimension at nose from wingcam
o_w2n = P*r_w2n/f;

% Display results
%--------------------------------------------------------------------
disp(sprintf('\nConstant parameters'));
disp(sprintf('-------------------'));
disp(sprintf('Horizontal resolution: %d', D_H));
disp(sprintf('Vertical resolution: %d', D_V));
disp(sprintf('Baseline: %2.2f cm', B*100));
disp(sprintf('Pixel size: %2.2f um', P*1000000));
disp(sprintf('Subpixel accuracy: %2.2f', p_i));
disp(sprintf('Focusing distance: %2.2f m', r_f));
disp(sprintf('F-stop number: f/%2.1f', a));
disp(sprintf('Circle of confusion diameter: %2.2f um', c*1000000));
disp(sprintf('Minimum pixels on target: %d', U));
disp(sprintf('Minimum number of disparities: %d', d_min));
disp(sprintf('Vehicle wingspan: %2.3f m', wingspan));
disp(sprintf('Vehicle body length: %2.3f m', body));

disp(sprintf('\nDerived parameters'));
disp(sprintf('-------------------'));
disp(sprintf('Horizontal angle of view: %2.3f degrees ', haov*(180/pi)));
disp(sprintf('Vertical angle of view: %2.3f degrees ', vaov*(180/pi)));
disp(sprintf('Focal length: %2.3f mm', f*1000));
disp(sprintf('Range to camera overlap: %2.3f m', r_co));
disp(sprintf('Minimum resolvable range: %2.3f m', r_min));
disp(sprintf('Maximum resolvable range: %2.3f m', r_max));
disp(sprintf('Range resolution at min range (%2.3fm): %2.3f m', r_min,rr_min));
disp(sprintf('Range resolution at max range (%2.3fm): %2.3f m', r_max,rr_max));
disp(sprintf('Range at which range resolution is 1cm: %2.5f cm', 100*r_rr1cm));
disp(sprintf('Smallest obstacle dimension at min range (%2.3fm): (%2.3f) cm', r_min,o_r_min*100));
disp(sprintf('Smallest obstacle dimension at max range (%2.3fm): (%2.3f) cm', r_max,o_r_max*100));
disp(sprintf('Smallest obstacle dimension at range r: (%2.5f)r cm', 100*o_min));
disp(sprintf('Hyperfocal distance: %2.3f m', h));
disp(sprintf('Near focus distance: %2.3f m', r_nf));
disp(sprintf('Far focus distance: %2.3f m', r_ff));
disp(sprintf('Maximum wing baseline: %2.3f m', B_wing));
disp(sprintf('Maximum body baseline: %2.3f m', B_body));
disp(sprintf('Wingcam to nose distance: %2.3f m', r_w2n));
disp(sprintf('Minimum dimension at nose from wingcam: %2.5f m', o_w2n));


% Plot achievable range
%--------------------------------------------------------------------
d = [d_min:d_max];
r_d = (B*f)./(d*P);
figure(1); subplot(2,3,1);
plot(r_d); grid on;
xlabel('Disparity (pixels)');
ylabel('Range (meters)');
axis([d_min d_max r_min r_max]);

r = [0.001:0.1:10000];
d_r = (B*f)./(r*P);
figure(1); subplot(2,3,2);
plot(r,d_r); grid on;
xlabel('Range (meters)');
ylabel('Disparity (pixels)');
axis([r_min r_max d_min d_max]);

% Parameters to match smallv.pdf plot
%B = 0.090;
%P = 0.0000075;
%p_i = (1/16);
ru_r = ((r.^2)/(B*f))*(P*p_i);

figure(1); subplot(2,3,3);
plot(r,ru_r); grid on;
xlabel('Range to object (meters)');
ylabel('Range Uncertainty (meters)');
axis([r_min r_max rr_min rr_max]);

figure(1); subplot(2,3,4);
ru_d = ((r_d.^2)/(B*f))*(P*p_i);
plot(d,ru_d); grid on;
xlabel('Disparity (pixels)');
ylabel('Range Uncertainty (meters)');
axis([d_min d_max rr_min rr_max]);

figure(1); subplot(2,3,5);
o_min_r = (o_min*100*[r]);
plot(r,o_min_r); grid on;
xlabel('Range (meters)');
ylabel('Minimum resolvable size (cm)');
axis([r_min r_max 100*o_r_min 100*o_r_max]);

figure(1); subplot(2,3,6);
fov_h = r*((P*D_H)/f);
plot(r,fov_h); grid on;
xlabel('Range (meters)');
ylabel('Horizontal field of view (m)');
axis([r_min r_max 0 r_max]);

%print -dtiff stereo_tradeoffs.tif


