function [] = demo_featuresim()
%--------------------------------------------------------------------
%
% Copyright (c) 2013 Jeffrey Byrne <jebyrne@gmail.com>
%
%--------------------------------------------------------------------

%% Inputs
fopt = featuresim.opts();
fopt.do_perturbation = false;  % no perturbation
fopt.do_quantization = false; % pixel quantization
fopt.W = 2456;  % cam width
fopt.H = 2048;  % cam height
fopt.fx = 2429.71515;  % cam focal length x
fopt.fy = 2429.71515;  % cam flocal elngth y
fopt.u0 = 1194.65883;  % cam principal point  
fopt.v0 = 1021.94076;  % cam principal point 
fopt.do_pointcloud = 1;
fopt.pointcloud_mode = 'rectgrid';


%% Camera pose (body) x = [X Y Z Roll Pitch Yaw]'
x_cam_in_bff = [0 0 0 0 -pi/2 0]';  % downward looking camera 
x_bff_in_ned = [0*randn(1,3) 0*randn(1,3)]';  % body fixed frame on vehicle
x_ned_in_enu = [0 0 featuresim.util.feet2meters(50) 0 0 0]';  % north-east-down vehicle in east-north-up world


%% Feature simulation 
[fph2,k_valid,K,P_enu2cam2,Q] = featuresim.camera(x_bff_in_ned, x_ned_in_enu, x_cam_in_bff, [], fopt);
[fph1,k_valid,K,P_enu2cam1] = featuresim.camera(x_bff_in_ned + [0 0 0 0.2*randn(1,3)]', x_ned_in_enu+[1+rand 1+rand randn 0 0 0]', x_cam_in_bff, [], fopt);
if (length(k_valid) < size(fph2,2))
  error('features outside field of view');
end

%% Triangulation
fprintf('[%s]: linear triangulation\n', mfilename);
T = [P_enu2cam2;0 0 0 1]/[P_enu2cam1;0 0 0 1];
R = T(1:3,1:3)';  
t = -T(1:3,4);
[Qhat_incam1] = featuresim.reconstruction.linear_triangulation(K,R,t,fph1,fph2)
Q_incam1 = P_enu2cam1*Q

mse = (Qhat_incam1 - Q_incam1).^2;  mse = mean(mse(:));
fprintf('[%s]: mse = %1.4f\n', mfilename, mse);


