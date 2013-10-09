function [R] = euler2dcm(euler)
%--------------------------------------------------------------------
%
% File: euler2dcm.m
% Author: r. k. prasanth, Jeffrey Byrne (jbyrne@ssci.com), 
%
% Description:  Compute the direction cosine matrix corresponding to the
% provided euler angles in the same format as provided by the Simulink
% block "Euler to DCM".  
%
% Inputs:
%   euler: 3x1 Euler angles (rad)
%
% Outputs:
%    R: 3x3 rotation matrix that matches the simulink block output.
%
% Copyright (c) 2013 Jeffrey Byrne <jebyrne@gmail.com>
%
%--------------------------------------------------------------------

% sines and cosines
cf = cos(euler(1)); sf = sin(euler(1));
ct = cos(euler(2)); st = sin(euler(2));
cp = cos(euler(3)); sp = sin(euler(3));

% rotation matrix
r1 = [1,0,0;0,cf,sf;0,-sf,cf];
r2 = [ct,0,-st;0,1,0;st,0,ct];
r3 = [cp,sp,0;-sp,cp,0;0,0,1];
R = r1*r2*r3;

       
