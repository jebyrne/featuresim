function [F,E] = fundamental_matrix(varargin)
%--------------------------------------------------------------------
%
% File: fundamental_matrix.m
%
% Description:  Compute the fundamental matrix (and optionally the
% essential matrix) given camera intrinsics and extrinsics. 
%
% Inputs: 
%   K: 3x3 intrinsic calibration matrix
%   R: 3x3 rotation matrix
%   t: 3x1 translation vector
%
% -OR-
% 
%   K: 3x3 intrinsic calibration matrix
%   T: 4x4 transformation matrix from camera 1 to camera 2
%
% Output:
%   F: 3x3 fundamental matrix
%   E: 3x3 essential matrix 
%
% Notes: 
%   Given two projection matrices P1=[I|0] and P2=[R|T], then the
%   essential matrix is E=skew(T)R.  If P1 is not the identity,
%   then we must first transform P1 to the identity, because P2
%   must be relative to P1 with P1 at the world origin.  Be careful
%   with defining R,T as inputs.  For example, we know (R12,T12)
%   which is camera two written in camera one.  We also know
%   (R21,T21) which is camera one written in camera two.  If
%   P1=[I|0], then P2=[R21|T21]=[R12|-R12*T12].
%
% Copyright (c) 2013 Jeffrey Byrne <jebyrne@gmail.com>
%
%--------------------------------------------------------------------
if (nargin == 3)
  K = varargin{1};
  R = varargin{2};
  t = varargin{3};
elseif (nargin == 2)
  K = varargin{1};
  T = varargin{2};
  R = T(1:3,1:3);
  t = T(1:3,4);
end
  
T_hat = featuresim.util.skew_symmetric(t);
E = T_hat*R;
invK = inv(K);
F = invK'*E*invK;

