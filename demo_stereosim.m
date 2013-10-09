%--------------------------------------------------------------------
%
% File: demo_stereosim.m
% Author: Jeffrey Byrne (jbyrne@gmail.com)
%
% Description:  Wrapper script to demonstrate and test 
% the stereo simulator of featuresim
%
%--------------------------------------------------------------------


%% Option initialization
% Stereo simulation options (ImageNav Applanix intrinsics)
stereo_opt.debug = 1;
stereo_opt.imscale = 1;
stereo_opt.W = 640;      % Width  (pixels)
stereo_opt.H = 480;      % Height (pixels)
stereo_opt.p = 0.009;     % Pixel size (millimeters)
stereo_opt.fx = 1000;   % Focal length (pixels)
stereo_opt.fy = 1000;   % Focal length (pixels)
stereo_opt.u0 = 320;  % Principal point x (pixels)
stereo_opt.v0 = 240;  % Principal point y (pixels)
stereo_opt.N = 4;
stereo_opt.do_plot = 1;
stereo_opt.do_epipolar = 1;
stereo_opt.do_quantization = 0;
stereo_opt.do_f2_perturbation = 0;
stereo_opt.do_eo = 0;
stereo_opt.pointcloud_mode = 'grid';
stereo_opt.QZ_std = 0;

% Stereo camera pose (body) x=[X Y Z Roll Pitch Yaw]'
x_bff_in_ned_1 = [randn randn randn featuresim.util.d2r(randn) featuresim.util.d2r(randn) featuresim.util.d2r(randn)]';
x_bff_in_ned_2 = [randn randn randn featuresim.util.d2r(randn) featuresim.util.d2r(randn) featuresim.util.d2r(randn)]';
x_ned_in_enu = [0 0 10 0 0 0]';       % Downward looking
x_cam_in_bff = [0 0 0 0 -pi/2 0]';     % Downward looking


%% Simulation! - Generate simulated stereo points from exterior orientation
[f1ph,f2ph,e1ph,e2ph,T_1to2,H_1to2,K] = featuresim.stereo(stereo_opt, x_bff_in_ned_1, x_bff_in_ned_2, x_ned_in_enu, []);




