%--------------------------------------------------------------------
%
% Tests to check epipolar projective geometry
%
% Copyright (c) 2006 Jeffrey Byrne <jebyrne@gmail.com>
%
%--------------------------------------------------------------------


% Prepare display
%------------------------------------------------
figure(1); clf; 
subplot(1,2,1); hold on; title('Image 1');
plot(50,50,'bo'); grid minor;
axis equal; axis([0 100 0 100]); 
subplot(1,2,2); hold on; title('Image 2');
plot(50,50,'bo'); grid minor;
axis equal; axis([0 100 0 100]); 

% Camera matrices
%-------------------------------------------------
K = [1 0 50; 0 1 50; 0 0 1];
%R = featuresim.util.FixedDeg2DCM(1,2,3);  % pry
R = featuresim.util.FixedDeg2DCM([2 1 3]);  % pry
T = [10 20 0]';

P1 = [eye(3) zeros(3,1)];
P2 = [R' -R'*T];

% Fundamental matrix
%------------------------------------------------
T_skew = [0 -T(3) T(2); T(3) 0 -T(1); -T(2) T(1) 0];
E = T_skew*R;
F = inv(K)'*E*inv(K);

% Epipolar line for point Q in image 1
%------------------------------------------------
Q = [10 0 1 1]';

% Q projects onto image 1 at x
x = (1/Q(3))*K*P1*Q;

% Epipolar line for x in image 2
l = F'*x;
u = [0:100];
v = (-l(1)*u - l(3))/l(2);
p = [u;(v);ones(1,length(v))];

% Display epipolar line
subplot(1,2,2); line(p(1,:),p(2,:));


% Ground truth projections
%------------------------------------------------
% Image 1 of Q
subplot(1,2,1);
plot(x(1)/x(3), x(2)/x(3), 'b+');

% Image 2 of Q given known camera transform
y = (1/Q(3))*K*P2*Q;
subplot(1,2,2); plot(y(1)/y(3), y(2)/y(3), 'g+');


% Iterate over Q distances
%------------------------------------------------
for vscale=[0.2 0.4 0.6 0.8 2 4 8]
  % Move X closer to camera along projection direction
  X = vscale*Q;
  X(4) = 1;

  % Project this to image 1
  x = (1/Q(3))*K*P1*Q;  
  subplot(1,2,1);
  plot(x(1)/x(3), x(2)/x(3), 'b+');
  
  % Project this to image 2
  y = (1/X(3))*K*P2*X

  % Plot to verify that this projection falls on epipolar line
  subplot(1,2,2); plot(y(1)/y(3), y(2)/y(3), 'r+');
end

