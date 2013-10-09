function img = pts2img(pts)
%--------------------------------------------------------------------
%
% File: pts2img.m
%
% Description:  Create a binary image from a sparse set of 2D points
%
% Copyright (c) 2013 Jeffrey Byrne <jebyrne@gmail.com>
%
%--------------------------------------------------------------------

% Create binary point images
img = sparse(round(pts(2,:)), round(pts(1,:)), ones(1,size(pts,2)));
%img1(1,1) = 0.0001;
%img1(1024,1024) = 0.0001;
img = full(img);
