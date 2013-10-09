function [P] = bff2cam(x_bff_in_ned, x_ned_in_enu, x_cam_in_bff)
%-------------------------------------------------------------------------
%
% File:    bff2stereo.m
% Date:	   December 30, 2008
%
% Copyright (c) 2008 SCIENTIFIC SYSTEMS COMPANY, INC.
%
% Description: Given poses of the body fixed frame (BFF) in the  
% north east down (NED) frame, the offset of the NED frame in the world
% east north up (ENU) frame, and the calibration or gimbal offset of the
% camera in the BFF, return the camerasim pose that will render the camera
% view consistent with this transformation chain.  Also, return the
% vector transformations from ENU to CAM and NED to CAM that will transform 
% a vector in ENU or NED to the camera frame.  
%
% Inputs: All poses x=[x y z roll pitch yaw]^T
%   x_bff_in_ned_1: Pose of body fixed frame (cam 1) in NED from telemetry
%   x_bff_in_ned_2: Pose of body fixed frame (cam 2) in NED from telemetry
%   x_ned_in_enu: Intial conditions of NED in ENU frame
%   x_cam_in_bff_1: Pose of camera 1 in body from calibration or gimbals
%   x_cam_in_bff_2: Pose of camera 2 in body from calibration or gimbals (optional)
%
% Outputs:
%   T_1to2: p_cam_2 = T_1to2*[p_cam_1; 1]
%   P1: 3x4 projection matrix from world to camera 1
%   P1: 3x4 projection matrix from world to camera 2
%
% All orientations are 3-2-1 Euler rotations, all angles radians
%
% Copyright (c) 2013 Jeffrey Byrne <jebyrne@gmail.com>
%
%-------------------------------------------------------------------------

% ENU to NED (body reference) transform
p_ned_in_enu = x_ned_in_enu(1:3);
T_enu2ned = featuresim.util.enu2ned(p_ned_in_enu);
R_enu2ned = T_enu2ned(1:3,1:3);

% NED (body reference) to Body Fixed Frame (BFF) transform (pose 1)
p_bff_in_ned = x_bff_in_ned(1:3);
w_bff_in_ned = x_bff_in_ned(4:6);
R_ned2bff = featuresim.util.euler2dcm(w_bff_in_ned);  % ZYX Euler Angles
T_ned2bff = [R_ned2bff -R_ned2bff*p_bff_in_ned; [0 0 0 1]];

% BFF to camera transform (BFF->CAM)=(X->Z,Y->X,Z->Y) (pose 1)
p_cam_in_bff = x_cam_in_bff(1:3);
w_cam_in_bff = x_cam_in_bff(4:6);
R_bff2cam = featuresim.util.euler2dcm(w_cam_in_bff);
T_bff2cam = [R_bff2cam -R_bff2cam*p_cam_in_bff; [0 0 0 1]];
T_bff2cam_permutation = [0 1 0 0; 0 0 1 0; 1 0 0 0; 0 0 0 1]; 
T_bff2cam = T_bff2cam_permutation*T_bff2cam;
R_bff2cam = T_bff2cam(1:3,1:3);

% ENU to camera transformation
T_enu2cam = T_bff2cam*T_ned2bff*T_enu2ned;

% Camera projection matrices
P = T_enu2cam(1:3,:);
