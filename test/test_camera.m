%--------------------------------------------------------------------
% 
% Copyright (c) 2013 Jeffrey Byrne <jebyrne@gmail.com>
%
%--------------------------------------------------------------------


%% Options
opt.W = 640;      % Width  (pixels)
opt.H = 480;      % Height (pixels)
opt.p = 0.009;     % Pixel size (millimeters)
opt.fx = 1000;   % Focal length (pixels)
opt.fy = 1000;   % Focal length (pixels)
opt.u0 = 320;  % Principal point x (pixels)
opt.v0 = 240;  % Principal point y (pixels)
opt.imscale = 1;  % image scale factor (should always be 1)
opt.do_plot = 1;
opt.do_quantization = 1;  % pixel quantization
opt.N = 16; % 3D point spacing
opt.pointcloud_mode = 'squaregrid';


%% Stereo camera pose (body) x=[X Y Z Roll Pitch Yaw]'
x_bff_in_ned = [randn randn randn featuresim.util.d2r(randn) featuresim.util.d2r(randn) featuresim.util.d2r(randn)]';
x_ned_in_enu = [0 0 10 0 0 0]';       % Downward looking
x_cam_in_bff = [0 0 0 0 -pi/2 0]';     % Downward looking


%% Simulation!
[fph] = featuresim.camera(x_bff_in_ned, x_ned_in_enu, x_cam_in_bff, [], opt);




