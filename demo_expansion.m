%--------------------------------------------------------------------
%
% File: test_stereo_simulator.m
% Author: Jeffrey Byrne (jbyrne@ssci.com)
%
% Description:  Wrapper script to demonstrate and test 
% the stereo simulator (stereo_simulator.m)
%
% $Id: ttc.m 15 2012-01-06 01:53:17Z jbyrne $
%
%--------------------------------------------------------------------

%% Option initialization
opt.do_plot = 1;
opt.do_epipolar = 1;
opt.do_quantization = 0;
opt.do_f2_perturbation = 0;
opt.do_eo = 0;
opt.pointcloud_mode = 'grid';


%% Stereo camera pose (body) x=[X Y Z Roll Pitch Yaw]'
x_bff_in_ned_1 = [0 0 0 0 0 0]';
x_bff_in_ned_2 = [0 0 -1 0 0 0]';
x_ned_in_enu = [0 0 2 0 0 0]';       % Downward looking
x_cam_in_bff = [0 0 0 0 -pi/2 0]';     % Downward looking


%% Simulation
[f1ph,f2ph,e1ph,e2ph,T_1to2,H_1to2,K] = featuresim.stereo(opt, x_bff_in_ned_1, x_bff_in_ned_2, x_ned_in_enu, []);




