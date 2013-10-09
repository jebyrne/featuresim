function [X,X_err] = optimal_triangulation(K,R12,T12,x1,x2)
%-------------------------------------------------------------------
%
% File: optimal_triangulation.m
%
% Description:  
%
% We are given two points p1 and p2 such that 3D point X projects
% to p1 in image 1 and p2 in image 2.  We are also given the (R,T)
% for camera two relative to camera one, which provides the
% projection matrices for each camera p1 = (P1)X and p2 = (P2)X.
% We wish to reconstruct the 3D point X such that the reprojection
% error (orthogonal distance from epipolar lines to correspondence)
% is minimized.  We use the optimal triangulation method of Hartley
% and Zisserman (Algorithm 12.1, p318)
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
%   X_err(i): |AX-b| 
%
% Copyright (c) 2008 Jeffrey Byrne <jebyrne@gmail.com>
%
%--------------------------------------------------------------------

X = [];
X_err = [];

% Compute fundamental matrix
F0 = fundamental_matrix(K,R12',-R12'*T12);

for n=1:size(x1,2)
  % (i) Translate x1 and x2 to origin
  T = [1 0 -x1(1,n);
       0 1 -x1(2,n);
       0 0 1];
  Tp = [1 0 -x2(1,n);
       0 1 -x2(2,n);
       0 0 1];  
  %x1n = inv(K)*x1(:,n);
  %x2n = inv(K)*x2(:,n);
  
  % (ii) Fundamental matrix already taken into account K
  %[F0] = fundamental_matrix(K,R12',-R12'*T12);
  F = inv(Tp')*F0*inv(T);

  % (iii) Compute left epipole: F*e1=0, right epipole F'*e2=0
  e = linear_least_squares(F);
  e = e ./ norm(e);
  ep = linear_least_squares(F');
  ep = ep ./ norm(ep);
  
  % (iv) Form matrices R (R1) and R' (R2)
  R = [e(1) e(2) 0;
       -e(2) e(1) 0;
       0 0 1];
  
  Rp = [ep(1) ep(2) 0;
        -ep(2) ep(1) 0;
        0 0 1];
  
  % (v) Replace F appropriately
  F = Rp*F*R';
  
  % (vi) Set variables
  f = e(3);
  fp = ep(3);
  a = F(2,2);
  b = F(2,3);
  c = F(3,2);
  d = F(3,3);
  
  % Polynomial: Using triangulation_roots to simplify (12.7)
  % g(t) = (-a^2*d*f^4*c+b*c^2*f^4*a)*t^6+(a^4-a^2*d^2*f^4+2*a^2*fp^2*c^2+fp^4*c^4+b^2*c^2*f^4)*t^5+(-a*d^2*f^4*b+4*a*b*fp^2*c^2+2*a*b*f^2*c^2+4*a^3*b+4*a^2*fp^2*c*d-2*a^2*f^2*c*d+4*fp^4*c^3*d+b^2*c*f^4*d)*t^4+(8*a*b*fp^2*c*d+6*fp^4*c^2*d^2+2*b^2*fp^2*c^2+6*a^2*b^2+2*b^2*f^2*c^2+2*a^2*fp^2*d^2-2*a^2*f^2*d^2)*t^3+(4*b^2*fp^2*c*d+4*a*b^3+2*b^2*f^2*c*d-2*a*b*f^2*d^2-a^2*d*c+4*a*b*fp^2*d^2+4*fp^4*c*d^3+b*c^2*a)*t^2+(-a^2*d^2+b^4+b^2*c^2+fp^4*d^4+2*b^2*fp^2*d^2)*t-a*d^2*b+b^2*c*d

  p = [(-a^2*d*f^4*c+b*c^2*f^4*a)
       (a^4-a^2*d^2*f^4+2*a^2*fp^2*c^2+fp^4*c^4+b^2*c^2*f^4)
       (-a*d^2*f^4*b+4*a*b*fp^2*c^2+2*a*b*f^2*c^2+4*a^3*b+4*a^2*fp^2*c*d-2*a^2*f^2*c*d+4*fp^4*c^3*d+b^2*c*f^4*d)
       (8*a*b*fp^2*c*d+6*fp^4*c^2*d^2+2*b^2*fp^2*c^2+6*a^2*b^2+2*b^2*f^2*c^2+2*a^2*fp^2*d^2-2*a^2*f^2*d^2)
       (4*b^2*fp^2*c*d+4*a*b^3+2*b^2*f^2*c*d-2*a*b*f^2*d^2-a^2*d*c+4*a*b*fp^2*d^2+4*fp^4*c*d^3+b*c^2*a)
       (-a^2*d^2+b^4+b^2*c^2+fp^4*d^4+2*b^2*fp^2*d^2)
       (-a*d^2*b+b^2*c*d)];
       
  % (vii) Find real part roots of polynomial
  t = real(roots(p));
  
  % (viii): Evaluate cost function 12.5 over all roots
  s = ((t.^2) ./ (1+(f.^2)*(t.^2))) + (((c.*t + d).^2)./((a.*t+b).^2 + (fp.^2)*(c.*t+d).^2));
  sinf = (1/(f^2)) + (c^2 / (a^2 + (fp^2)*(c^2)));
  [smin,smin_index] = min(s);
  if (smin > sinf)
    error('tinf minimum');
  else
    tmin = t(smin_index);
  end
  
  % (ix): Evaluate two lines at tmin
  l = [tmin*f 1 -tmin]';
  lp = [-fp*(c*tmin+d) a*tmin+b c*tmin+d]';
  xhat = [-l(1)*l(3) -l(2)*l(3) (l(1)^2)+(l(2)^2)]';
  xhatp = [-lp(1)*lp(3) -lp(2)*lp(3) lp(1)^2+lp(2)^2]';
  
  % (x): Transfer back to homogeneous pixel coordinates
  xhat = inv(T)*R'*xhat;
  xhatp = inv(Tp)*Rp'*xhatp;
  xhat = xhat ./ xhat(3);
  xhatp = xhatp ./ xhatp(3);  
  
  % (xi): Linear triangulation
  [X(:,n),X_err(n)] = linear_inh_triangulation(K,R12,T12,xhat,xhatp);

  % TESTING
  % [F,E] = fundamental_matrix(K,R12',-R12'*T12);
  % op = epipolar_projection(F,xhat,xhatp);
  
end

