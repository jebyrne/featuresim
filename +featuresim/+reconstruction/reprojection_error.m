function [ssd_error] = reprojection_error(F,x,y)
%--------------------------------------------------------------------
%
% File: reprojection_error.m
%
% Description:  Given two sets of corresponding points, x in image
% 1 and y in image 2, and an estimate of rotation and translation
% between images as encoded in the fundamental matrix, compute the
% sum of squared lengths of the orthogonal projections of points y
% onto the epipolar lines
%
% Inputs: 
%   F: estimated fundamental matrix
%   x(:,i): homogeneous points in image 1
%   y(:,i): homogeneous points in image 2
% 
% Output:
%   ssd_error: sum of squared error between y and l
%
% Copyright (c) 2008 Jeffrey Byrne <jebyrne@gmail.com>
%
%--------------------------------------------------------------------

% Iterate over all corresponding points
ssd_error = 0;
for n=1:length(x)
  ssd_error = ssd_error + epipolar_projection(F,x(:,n),y(:,n));  
end

