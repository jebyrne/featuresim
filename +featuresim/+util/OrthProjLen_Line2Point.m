function [len,n] = OrthProjLen_Line2Point(l,p)
%--------------------------------------------------------------------
%
% File: OrthProjLen_Line2Point
%
% Description:  Given a line defined by homogeneous coefficients l
% and a point p, compute the length of the orthogonal projection
% from the point to the line.  
%
% Inputs: 
%   l: homogeneous coefficient vector of line (column)
%   p: homogeneous point (column)
%
% Outputs
%   len: length of the orthogonal projection from line l to point p
%   n: unit vector orthogonal to line l
%
% Copyright (c) 2013 Jeffrey Byrne <jebyrne@gmail.com>
%
%--------------------------------------------------------------------

% TESTING
%len = sqrt(((p'*l).^2) ./ (norm(skew([0 0 1])*l).^2));
% a = repmat(norm(skew_symmetric([0 0 1])*l),size(p,2),1);
% len = sqrt(((p'*l).^2) ./ (a.^2));
a = sqrt(l(1)*l(1)+l(2)*l(2)); % same as norm(skew_symmetric([0 0 1])*l)
len = abs((p'*l) ./ a); % for matrix p, consider len = abs((l'*p) ./ a)';
n = zeros(size(p));
return;

% Line parameterization:
%   General form: ax +by +c = 0  
%   Parameterized form: d = x*cos(t) + y*sin(t)
%      Constraints: n = [cos(t) sin(t)]; norm(n)=1

% Force d positive
if (l(3) < 0)
  v_ab = l(1:2);
  c = -l(3);
else
  v_ab = -l(1:2);
  c = l(3);
end

% Error check
if (norm(v_ab) == 0)
  % Invalid (?) line
  len = 0;
  n = [0 0];
  return;
end

% Compute orthogonal unit vector from origin to line
% Unit normal orthogonal to line (theta)
n = v_ab./norm(v_ab);

% Compute orthogonal vector magnitude from origin to line
d = c./norm(v_ab);

% Compute angle between line and point
%len1 = norm(p)*((p(1:2)'*n)/(norm(p))) - d

% Project onto the unit normal vector.  Both P and n are defined
% relative to the same origin. 
a = p(1:2)'*n;

% Compute relative length of projection and unit normal distance
%len = abs(a - d);
len = (a - d);

% MASKS: p129 
%d = sqrt(((p'*l).^2) ./ (norm(skew([0 0 1])*l).^2));
%if abs((d - abs(len)) > 0.0000001)
%  d - abs(len)
%  error('orthogonal projection mismatch')
%end




