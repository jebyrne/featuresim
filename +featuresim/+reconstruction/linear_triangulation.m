function [X,X_err] = linear_triangulation(K,R12,T12,x1,x2)
%-------------------------------------------------------------------
%
% File: linear_triangulation.m
%
% Description:  
%
% We are given two points p1 and p2 such that 3D point X projects
% to p1 in image 1 and p2 in image 2.  We are also given the (R,T)
% for camera two relative to camera one, which provides the
% projection matrices for each camera p1 = (P1)X and p2 = (P2)X.
% We wish to reconstruct the 3D point X from the projections. 
% If we know the projections satisfy epipolar constraints, then we
% can compute X linearly (H+Z, p312).  This follows the homogeneous
% approach in Hartley+Zisserman
%
% Inputs:
%   K: intrinsic calibration matrix
%   R12: rotation from camera 1 to camera 2
%   T12: translation from camera 1 to camera 2
%   x1(:,i): homogeneous point in pixel coordinates
%   x2(:,i): homogeneous point in pixel coordinates
%   
% Outputs:
%   X(:,i): non-homogeneous 3D point written in camera 1 
%   X_err(i): |AX| 
%
% Copyright (c) 2013 Jeffrey Byrne <jebyrne@gmail.com>
%
%--------------------------------------------------------------------

% Create projection matrix for camera 1
P1 = [eye(3) zeros(3,1)];

% Create projection matrix for camera 2
P2 = [R12' -R12'*T12];

N = size(x1,2);
for n=1:N
  % Projection of X in camera 2
  v = x2(:,n);

  % Projection of X in camera one
  u = x1(:,n);
  
  % Linear triangulation: AX=0;
  % Linear least squares solution X  (H+Z p312)
  PK1 = K*P1;
  PK2 = K*P2;  
  A = [u(1)*PK1(3,:) - PK1(1,:);
       u(2)*PK1(3,:) - PK1(2,:);
       v(1)*PK2(3,:) - PK2(1,:);
       v(2)*PK2(3,:) - PK2(2,:)];
  [U,S,V] = svd(A);
  X(:,n) = V(1:3,end) ./ V(4,end);
  X_err(n) = norm(A*[X(:,n);1]);
end
