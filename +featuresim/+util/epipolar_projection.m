function [op,n] = epipolar_projection(F,x1,x2)
%--------------------------------------------------------------------
%
% File: epipolar_projection.m
%
% Description:  Given a fundamental matrix, compute the epipolar
% line corresponding to point x1 in image 1 in image 2.  Next,
% compute the orthogonal projection of the point x2 in image 2 onto
% the epipolar line.  The length of the orthogonal projection is
% the epipolar projection.  Note that "image 1" and "image 2"
% depend on the definition of the fundamental matrix.
%
% Inputs:
%   F: Fundamental matrix
%   x1: 3x1 homogeneous points in pixel coordinates in image 1
%   x2: 3xN homogeneous points in pixel coordinates in image 2
% Outputs:
%   op: orthogonal projection length (Nx1)
%   n: orthogonal projection direction (unit vector normal to
%   epipolar line)
%
% Copyright (c) 2013 Jeffrey Byrne <jebyrne@gmail.com>
%
%--------------------------------------------------------------------

% Epipolar line l2 in image 2 corresponding to feature x1 in image 1
l2 = F*x1;

% Length of orthogonal projection of feature x2 onto epipolar line
%for k=1:size(x1,2)
%  [op(k) n(:,k)] = OrthProjLen_Line2Point(l2(:,k),x2(:,k));
%end

[op, n] = featuresim.util.OrthProjLen_Line2Point(l2,x2);
