function T = enu2ned(O_ned_in_enu)
%--------------------------------------------------------------------
%
% File: enu2ned.m
%             
% Description:  Return transformation matrix that will convert a vector in
% ENU to a vector in NED.  The origin of NED is coincident with the body
% frame, and the orientation of the NED and the ENU are aligned after the
% initial axis permutation. 
%
% See also:
%   1. Grewal. Weill, Andrews, "Global Positioning Systems, Inertial Navigation,
%   and Integration", C.73  p.338 
%   2. Craig, "Introduction to Robotics, Second Edition", p39 (2.45) 
%   3. ImageNavSim README for frame description
%    
% Copyright (c) 2008 Jeffrey Byrne <jebyrne@gmail.com>
%
%--------------------------------------------------------------------
A = [0 1 0; 1 0 0; 0 0 -1];
T = [A -A*O_ned_in_enu; 0 0 0 1];

