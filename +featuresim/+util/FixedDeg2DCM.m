function [R] = Fixed2DCM(W)
%--------------------------------------------------------------------
%
% File: Fixed2DCM.m
%
% Description:  Fixed angles in degrees to direction cosine
% matrix.  Refer to Craig, "Introduction to Robotics", p46 (2.64) for
% variable naming and equations.
%
% Inputs:
%   W(1) g: roll angle
%   W(2) b: pitch angle
%   W(3) a: yaw angle
%
% Outputs
%   R: direction cosine matrix
% 
% Copyright (c) 2013 Jeffrey Byrne <jebyrne@gmail.com>
%
%--------------------------------------------------------------------

% Vector to scalar conversion
g = featuresim.util.d2r(W(1));
b = featuresim.util.d2r(W(2));
a = featuresim.util.d2r(W(3));

% Direction cosine matrix
R(1,1) = cos(a)*cos(b);
R(1,2) = cos(a)*sin(b)*sin(g)-sin(a)*cos(g);
R(1,3) = cos(a)*sin(b)*cos(g)+sin(a)*sin(g);
R(2,1) = sin(a)*cos(b);
R(2,2) = sin(a)*sin(b)*sin(g)+cos(a)*cos(g);
R(2,3) = sin(a)*sin(b)*cos(g)-cos(a)*sin(g);
R(3,1) = -sin(b);
R(3,2) = cos(b)*sin(g);
R(3,3) = cos(b)*cos(g);

