function K = get_K(cam_prms);
%--------------------------------------------------------------------
%
% File: get_K.m
%             
% Description: Get camera intrinsic calibration matrix from intrinsics.
%
% Inputs
%   cam_prms: camera parameters structure (get_camera_parameters.m)
%
% Output
%   K: 3x3 camera intrinsic calibration matrix 
%
% Copyright (c) 2008 Jeffrey Byrne <jebyrne@gmail.com>
%
%--------------------------------------------------------------------

% Input check
if (nargin ~= 1)
  error('[get_K]: Invalid input');
end
if (isstruct(cam_prms) == 0)
  error('[get_K]: Invalid input');
end  

% Intrinic calibration matrix
K = [cam_prms.fx 0 cam_prms.u0; 0 cam_prms.fy cam_prms.v0; 0 0 1];
