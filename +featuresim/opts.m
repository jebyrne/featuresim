function [opt] = opts()
%--------------------------------------------------------------------
%
% Copyright (c) 2008 Jeffrey Byrne <jebyrne@gmail.com>
%
%--------------------------------------------------------------------

%% Default simulator options
opt.debug = 1;
opt.do_plot = 1;
opt.do_epipolar = 1;
opt.imscale = 1;


%% Noise options
opt.do_quantization = 1;  % pixel quantization effects
opt.do_perturbation = 1;  % feature noise 
opt.do_eo = 0;  % randomly generate camera exterior orientation


%% Gaussian perturbation of features
opt.f_std = 0.5;
opt.f_mean = 0;


%% Gaussian perturbation for exterior orientation
opt.eo_t_std = 0.3;
opt.eo_t_mean = [0 0 10]';
opt.eo_w_std = 0.01;
opt.eo_w_mean = [0 0 0]';
opt.W_std = [0 0 0]';
opt.W1_std = [0 0 0]';
opt.W2_std = [0 0 0]';
opt.T_std = [0 0 0]';
opt.T1_std = [0 0 0]';
opt.T2_std = [0 0 0]';


%% Intrinsic camera parameters
%
% Normalized Image frame: [u(right),v(up),1] unit focal length *in
% front* of pinhole
%
%
opt.W = 640;      % Width  (pixels)
opt.H = 480;      % Height (pixels)
opt.p = 0.009;             % Pixel size (millimeters)
opt.fx = 12./opt.p; % Focal length x (pixels)
opt.fy = opt.fx; % Focal length y (pixels)
opt.u0 = 320;     % Principal point x (pixels)
opt.v0 = 240;     % Principal point y (pixels)
opt.K = [];  % intrinsic calibration matrix
opt.EO = [];  % exterior orientation (camera pose) 


%% 3D scene point properties
opt.do_pointcloud = 0;  % generate pointclouds at runtime
opt.QZ_std = 200;  % 3D point standard deviation
opt.pointcloud_mode = 'planar';  % see featuresim.camera for allowable modes
opt.N = 50; % number of 3D points

