function [rmat,sc,rderv,r1,r2,r3] = euler2rotmat(euler)
%--------------------------------------------------------------------
% 
% calculate rotation matrix from inertial to body axis 
% from Euler angles (3-2-1)
%
% usage: rmat = euler2rotmat(euler);
%
% inputs: euler - euler angles
%
% outputs: rmat - rotation matrix to go from inertial to body axis
%
% written by r. k. prasanth
% date: september 2000
%
% Copyright (c) 2008 Jeffrey Byrne <jebyrne@gmail.com>
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
rmat = r1*r2*r3;
       
% sines and cosines
sc = [sf, cf; st, ct; sp, cp];

% derivative of rotation matrix w.r.t euler angles and return
% as block-column matrix of size 3*3 by 3
if nargout > 2,
    r1d = [0,0,0;0,-sf,cf;0,-cf,-sf];
    r2d = [-st,0,-ct;0,0,0;ct,0,-st];
    r3d = [-sp,cp,0;-cp,-sp,0;0,0,0];
    rderv = [r1d*r2*r3; r1*r2d*r3; r1*r2*r3d];
end

