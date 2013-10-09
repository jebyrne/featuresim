function [V_hat] = vec2skew(V)
%--------------------------------------------------------------------
%
% File: vec2skew.m
%
% Description:  Convert a 3x1 vector V to a 3x3 skew symmetric
% matrix V_hat such that for any 3x1 vector Y: (V x Y) = (V_hat)(Y)
%
% Inputs: 
%   V: 3x1 vector
% 
% Output:
%   V_hat: 3x3 cross product (skew symmetric) matrix
%
% Copyright (c) 2008 Jeffrey Byrne <jebyrne@gmail.com>
%
%--------------------------------------------------------------------
V_hat = [0 -V(3) V(2); V(3) 0 -V(1); -V(2) V(1) 0];
