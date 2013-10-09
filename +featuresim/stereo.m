function [f1ph,f2ph,e1ph,e2ph,T_1to2,H_1to2,K,Q,valid1,valid2] = stereo(optin, x_bff_in_ned_1, x_bff_in_ned_2, x_ned_in_enu, Q, x_cam1_in_bff, x_cam2_in_bff)
%--------------------------------------------------------------------------
%
% File: featuressim.stereo
% Author: Jeffrey Byrne (jbyrne@gmail.com)
%
% Description:  Given 3D points in the world, two cameras with 
% known exterior orientation and intrinsics, project the 3D points 
% to the images.  There are many many useful options to set in here.
%
% See test_stereo_simulator.m for example usage and demonstration.
%
% Inputs: All poses x=[X,Y,Z,Roll,Pitch,Yaw]^T
%   optin: options structure (see test_stereo_simulator.m)
%   x_bff_in_ned_1: pose of body fixed frame 1 in north east down (NED)
%   x_bff_in_ned_2: pose of body fixed frame 2 in north east down (NED)
%   x_ned_in_enu: pose of NED in east north up (ENU) world frame
%   Q (optional): 4xN matrix of 3D points in ENU
%   x_cam_in_bff_1 (optional): pose of camera 1 base in body fixed frame 1
%   x_cam_in_bff_2 (optional): pose of camera 2 base in body fixed frame 2
%     Note that camera base poses are subsequently mapped by bff2stereo
%     from camera base frame XYZ axes to actual camera ZXY axes; e.g. given
%     default x_cam_in_bff euler angles of [0 -pi/2 0], the camera X axis
%     is aligned with body Y (right wing), camera Y is aligned with body -X
%     (out tail), and camera/body Z axes are aligned (boresight downward).
%
% Outputs:
%   f1ph: 3xN features (f) in image one (1), pixel coordinates (p), dehomogenized (h)
%   f2ph: 3xN features (f) in image two (2), pixel coordinates (p), dehomogenized (h)
%   e1ph: 3x1 epipole in image one, pixel coordinates (p), dehomogenized (h)
%   e2ph: 3x1 epipole in image two, pixel coordinates (p), dehomogenized (h)
%   T_1to2: 4x4 Transformation matrix from cam1 to cam2 p2 = T_1to2*p1
%   H_1to2: 4x4 Rotational Homography cam1 to cam2 
%   K: 3x3 intrinsic calibration matrix
%
% Frames:
%   CAM: X (right), Y (down), Z (optical axis, out).
%   BFF: X (out nose), Y (right), Z (down)
%   NED: X (out nos, north), Y (right, east), Z (down) (BFF attitude reference)
%   ENU: X (east), Y (north), Z (up).  Mapping/Global frame
% 
% Copyright (c) 2006 Jeffrey Byrne <jebyrne@gmail.com>
%
%--------------------------------------------------------------------------

% Input check
%-------------------------------------------------
if (nargin < 5)
  Q = [];
end
if (nargin < 6)
  % Point camera base X along body Z axis, aligning camera & body Z axes
  x_cam1_in_bff = [0 0 0 0 -pi/2 0]'; % (Downward looking)
end
if (nargin < 7)
  x_cam2_in_bff = x_cam1_in_bff;
end
if (nargin > 7)
  error('Invalid input');
end
if (isempty(Q) == 1)
  % Force pointcloud generation if no points are provided
  optin.do_pointcloud = 1;
else
  % Check size 4xN
  if (size(Q,1) ~= 4)
    error('Q must be 4xN');    
  end
end  

% Options
%-------------------------------------------------
% Default options
default_opt.debug = 1;
default_opt.do_plot = 1;
default_opt.do_epipolar = 1;
default_opt.N = 50;
default_opt.imscale = 1;

% Noise options
default_opt.do_quantization = 0;
default_opt.do_f2_perturbation = 0;
default_opt.do_outliers = 0;
default_opt.do_eo = 0;

% Gaussian perturbation for image 2 features 
default_opt.f2_std = 1;
default_opt.f2_mean = 0;
default_opt.f_std = 0.25;
default_opt.f_mean = 0;

% Gaussian perturbation for exterior orientation
default_opt.eo_t_std = 0.3;
default_opt.eo_t_mean = [0 0 10]';
default_opt.eo_w_std = 0.01;
default_opt.eo_w_mean = [0 0 0]';
default_opt.W_std = [0 0 0]';
default_opt.W1_std = [0 0 0]';
default_opt.W2_std = [0 0 0]';
default_opt.T_std = [0 0 0]';
default_opt.T1_std = [0 0 0]';
default_opt.T2_std = [0 0 0]';


% Default number of outliers
default_opt.num_outliers = 0;
default_opt.outlier_mean = [0 0]';
default_opt.outlier_std = [10 10]';

% Default camera parameters
%
% Normalized Image frame: [u(right),v(up),1] unit focal length *in
% front* of pinhole
%
default_opt.W = 640;      % Width  (pixels)
default_opt.H = 480;      % Height (pixels)
default_opt.p = 0.009;             % Pixel size (millimeters)
default_opt.fx = 12./default_opt.p; % Focal length (pixels)
default_opt.fy = default_opt.fx; % Focal length (pixels)
default_opt.u0 = 320;     % Principal point x (pixels)
default_opt.v0 = 240;     % Principal point y (pixels)
default_opt.K = [];
default_opt.EO = [];

% Default 3D point properties
default_opt.do_pointcloud = 0;
default_opt.QZ_std = 200;
default_opt.pointcloud_mode = 'planar';


% Set options: Overwrite defaults with provided options
opt = featuresim.util.get_options(optin, default_opt, -1);


% Intrinsic camera parameters
%-------------------------------------------------
% Image width (columns) in pixels
W = round(opt.W*opt.imscale);

% Image height (rows) in pixels
H = round(opt.H*opt.imscale);

% Pixel size (mm)
p = opt.p*(1/opt.imscale);

% Focal length (pixels)
f = opt.fx*opt.imscale;

% Center of projection (u0,v0)
u0 = opt.u0*opt.imscale;
v0 = opt.v0*opt.imscale;

% Intrinsic calibration matrix
if (isempty(opt.K) == 1)
  % Pixel frame: [u(right),v(down)] origin in upper left corner
  K = featuresim.util.get_K(opt);
else
  K = opt.K;
end


% Camera pose: Exterior orientation
%----------------------------------------------------
% Randomly generate exterior orientation (x) if requested
if (opt.do_eo == 1)
  x_bff_in_ned_1 = opt.eo_t_mean + opt.eo_t_std.*randn(6,1);
  x_bff_in_ned_2 = opt.eo_t_mean + opt.eo_t_std.*randn(6,1);
end

% Camera pose to relative exterior orientation transformation
[T_1to2, P_enu2cam_1, P_enu2cam_2] = featuresim.util.bff2stereo(x_bff_in_ned_1, x_bff_in_ned_2, x_ned_in_enu, x_cam1_in_bff, x_cam2_in_bff);

% Fundamental and Essential matrix
%------------------------------------------------
[F,E] = featuresim.util.fundamental_matrix(K, T_1to2);

% Epipoles 
%------------------------------------------------
% Origin of camera 1 in ENU
T_enu2cam_1 = [P_enu2cam_1; 0 0 0 1];
T_cam2enu_1 = inv(T_enu2cam_1);
o_cam_in_enu_1 = T_cam2enu_1*[0 0 0 1]';
o_cam_in_enu_1h = o_cam_in_enu_1(1:3) ./ o_cam_in_enu_1(4);

% Origin of camera 2 in ENU
T_enu2cam_2 = [P_enu2cam_2; 0 0 0 1];
T_cam2enu_2 = inv(T_enu2cam_2);
o_cam_in_enu_2 = T_cam2enu_2*[0 0 0 1]';
o_cam_in_enu_2h = o_cam_in_enu_2(1:3) ./ o_cam_in_enu_2(4);

% Epipole in image 1 (projection of origin of camera 2)
e1 = K*P_enu2cam_1*o_cam_in_enu_2;
e1ph = e1(1:2)./e1(3);
%e1ph = e1;

% Epipole in image 2 (projection of origin of camera 1)
e2 = K*P_enu2cam_2*o_cam_in_enu_1;
e2ph = e2(1:2)./e2(3);
%e2ph = e2;

% Rotational homography (camera 1 to camera 2)
%------------------------------------------------
R_1to2 = T_1to2(1:3,1:3);
H_1to2 = K*R_1to2*inv(K);

% Prepare display
%------------------------------------------------
% Image projections are in image coordinates (u,v)=(right,down)
% Origin: (u0,v0)=upper left
if (opt.do_plot == 1)
  % Left image
  figure(1); clf; 
  subplot(1,3,1); hold on; title('Image 1'); 
  
  % Epipole (focus of expansion)
  plot(e1ph(1),e1ph(2),'bo'); 
  axis equal; axis([1 W 1 H]); grid on;
  
  % Convert plot to image coordinates
  axis ij; set(gca,'XAxisLocation','top');
 
  % Right image
  subplot(1,3,2); hold on; title('Image 2'); 

  % Epipole 
  plot(e2ph(1),e2ph(2),'bo'); axis ij;
  axis equal; axis([1 W 1 H]); grid on;
  
  % Convert plot to image coordinates
  axis ij; set(gca,'XAxisLocation','top');
  
  % 3D point cloud
  subplot(1,3,3); hold on; title('3D Point Cloud');   
  grid on;
end


% Generate 3D points in ENU
%-------------------------------------------------------------
if (opt.do_pointcloud == 1)
  if ~isfield(opt,'QX_std'), opt.QX_std = opt.QZ_std; end
  if ~isfield(opt,'QY_std'), opt.QY_std = opt.QZ_std; end
  switch(opt.pointcloud_mode)
   case 'orthographic'
    % Model imagenav data
    Q = [400*rand(2,opt.N)-200; 20*rand(1,opt.N); ones(1,opt.N)];
   
   case 'random_cube'
    % Uniform random points in cube
    %Q = [400*rand(2,opt.N)-200; opt.QZ_std*rand(1,opt.N); ones(1,opt.N)];
    Q = [opt.QX_std*(rand(1,opt.N)-0.5); opt.QY_std*(rand(1,opt.N)-0.5); opt.QZ_std*(rand(1,opt.N)-0.5); ones(1,opt.N)];
    Q(1:3,:) = Q(1:3,:) - repmat(mean(Q(1:3,:),2),1,opt.N); % make zero-mean

   case 'gaussian'
    % 3D (box) gaussian
    Q = [opt.QX_std*randn(1,opt.N); opt.QY_std*randn(1,opt.N); opt.QZ_std*randn(1,opt.N); ones(1,opt.N)];
    Q(1:3,:) = Q(1:3,:) - repmat(mean(Q(1:3,:),2),1,opt.N); % make zero-mean

   case 'planar'
    % Generate random 3D points on plane
    %Q = [200*(2*rand(2,opt.N)-1); zeros(1,opt.N) ; ones(1,opt.N)];
    Q = [opt.QX_std*(rand(1,opt.N)-0.5); opt.QY_std*(rand(1,opt.N)-0.5); zeros(1,opt.N); ones(1,opt.N)];

   case 'biplanar'
    % Generate random 3D points on 2 planes
    Q = [2*rand(2,opt.N)-1; [ones(1,opt.N/2) 2*ones(1,opt.N/2)] ; ones(1,opt.N)];   
   
   case 'grid'
    % Grid points: (m)
    Q = [-1 1 1 1;
         0 1 1 1;
         1 1 1 1;
         -1 0 1 1;
         0 0 1 1;     
         1 0 1 1;          
         -1 -1 1 1;
         0 -1 1 1;     
         1 -1 1 1;               
         -1 -2 1 1;
         0 -2 1 1;     
         1 -2 1 1]';
    opt.N = size(Q,2);
    Q(1:2,:) = 1*Q(1:2,:);
    Q(3,:) = zeros(1,opt.N);
   
   otherwise
    error('Undefined pointcloud mode ''%s''',opt.pointcloud_mode);
  end
else
  opt.N = size(Q,2);
end

% Display point cloud
if (opt.do_plot == 1)
  figure(1);subplot(1,3,3); plot3(Q(1,:),Q(2,:),Q(3,:),'r+'); hold on;
  plot3(0,0,0,'bo');
  xlabel('X');ylabel('Y');zlabel('Z');
  
  % Display camera centers of projection
  plot3(o_cam_in_enu_1h(1),o_cam_in_enu_1h(2),o_cam_in_enu_1h(3),'bs','MarkerSize',20);
  text(o_cam_in_enu_1h(1),o_cam_in_enu_1h(2),o_cam_in_enu_1h(3),'1','Color',[0 0 1]);
  plot3(o_cam_in_enu_2h(1),o_cam_in_enu_2h(2),o_cam_in_enu_2h(3),'bs','MarkerSize',20);
  text(o_cam_in_enu_2h(1),o_cam_in_enu_2h(2),o_cam_in_enu_2h(3),'2','Color',[0 0 1]);

  % Plot massage
  view(3); axis equal;  
end


% Perspective projection
%------------------------------------------------
% Project to image 1 in image coordinates
x = K*P_enu2cam_1*Q;
xi_pos = x(3,:)>0; % flag 'valid' points (in front of camera)
x = x ./ repmat(x(3,:),3,1);

% Quantization scale factor
qscale = 1;
if isfield(opt,'quantization_factor'),
  qscale = opt.quantization_factor;
end

% Round to nearest pixel
x_unquantized = x;
if (opt.do_quantization == 1)
  x = round(x/qscale)*qscale;
  %x = round(x);
end

% Display projection
if (opt.do_plot == 1) 
  subplot(1,3,1); plot(x_unquantized(1,:), x_unquantized(2,:), 'b+');
  subplot(1,3,1); plot(x(1,:), x(2,:), 'r+');  
end

% Project to image 2 in image coordinates
y = K*P_enu2cam_2*Q;
yi_pos = y(3,:)>0; % flag 'valid' points (in front of camera)
y = y ./ repmat(y(3,:),3,1);

% Round to nearest pixel
y_unquantized = y;
if (opt.do_quantization == 1)
  y = round(y/qscale)*qscale;
  %y = round(y);
end

% Display projection
if (opt.do_plot == 1)
  subplot(1,3,2); plot(y_unquantized(1,:), y_unquantized(2,:), 'b+');  
  subplot(1,3,2); plot(y(1,:), y(2,:), 'r+');
end

% Draw epipolar lines in image 2
if ((opt.do_plot == 1) && (opt.do_epipolar == 1))
  subplot(1,3,2); 
  for n=1:opt.N
    % Epipolar line for x (image coordinates 1) in image 2
    l = F*x(:,n);
    
    % Is x an epipole? F'e = 0; 
    if (sum(l < eps('single')) == 3)
      % Don't display
      continue;
    end
    
    % Compute points along the epipolar line 
    if (l(2) == 0)
      % Vertical lines
      u = (-l(3)/l(1))*ones(1,W);
      v = [1:W];
    else
      % Non-vertical lines
      u = [1:W];
      v = (-l(1)*u - l(3))/l(2);
    end
    p = [u;v;ones(1,length(v))];
    
    % Display epipolar line
    line(p(1,:),p(2,:),'Color',[0 1 1]);
  end
end


% Gaussian feature tracking noise
%------------------------------------------------
% Perturb image 2 features after projection 
if (opt.do_f2_perturbation == 1)
  dy = opt.f2_mean + opt.f2_std.*randn(2,length(y));
  y(1:2,:) = y(1:2,:) + dy;

  % Overlay noisy features with ground truth
  if (opt.do_plot == 1)
    subplot(1,3,2); plot(y(1,:), y(2,:), 'r+');
  end
end


% Epipolar statistics 
%------------------------------------------------
% Compute the epipolar projection for each feature
if (opt.do_epipolar)
  op = [];
  for n=1:length(x)
    x_n = x(:,n);
    y_n = y(:,n);
    new_op = featuresim.util.epipolar_projection(F,x_n,y_n);
    op = [op new_op];
  end
  mean_op = mean(op);
  std_op = std(op);
end

if 0
  % Triangulation
  %------------------------------------------------
  % We are given two points p1 and p2 such that 3D point X projects
  % to p1 in image 1 and p2 in image 2.  We are also given the (R,T)
  % for camera two relative to camera one, which provides the
  % projection matrices for each camera p1 = (P1)X and p2 = (P2)X.
  % We wish to reconstruct the 3D point X from the projections. 
  % If we know the projections satisfy epipolar constraints, then we
  % can compute X linearly as follow (H+Z, p312)
  %
  % Triangulation of correspondence (x->y) in camera 1
  %[X,X_err] = linear_triangulation(K,R12,T12,x,y);
  %[X,X_err] = linear_inh_triangulation(K,R12,T12,x,y);
  [X,X_err] = optimal_triangulation(K,R12,T12,x,y);

  % Transform of points Q in world frame to camera frame 1
  Q1 = P1*Q;

  % Reprojection error
  for n=1:opt.N
    % Reprojection error to verify camera projections
    QX_err(n) = norm(X(:,n) - Q1(:,n));
    if (QX_err(n) < 0.01)
      %disp(sprintf('Reprojection error[%d]: %e', n, QX_err(n)));    
      if (opt.do_plot == 1)
        %subplot(1,3,3); hold on; plot3(X(1),X(2),X(3),'g+');
      end
    end
  end

end % triangulation

% Outliers
%------------------------------------------------
if (opt.do_outliers == 1)
  % Generate random correspondences
  xo = [];
  yo = [];
  for n=1:opt.num_outliers
    % Choose one feature at random
    m = floor((size(x,2)-1)*rand(1))+1;
    
    % Perturb it drastically
    y(1:2,m) = y(1:2,m) + opt.outlier_mean + opt.outlier_std.*randn(2,1);
  end
  if (opt.do_quantization == 1)
    y = round(y/qscale)*qscale;
    %y = round(y);
  end
end

% Enforce field of view
%------------------------------------------------
xi_fov = (x(1,:) >= 1) & (x(1,:) <= W) & (x(2,:) >= 1) & (x(2,:) <= H);
yi_fov = (y(1,:) >= 1) & (y(1,:) <= W) & (y(2,:) >= 1) & (y(2,:) <= H);

% Apply positive depth constraints (points behind camera are not visible)
xi_fov = xi_fov & xi_pos;
yi_fov = yi_fov & yi_pos;

% Feature outputs
%------------------------------------------------
%f1ph = x(:,xi_fov);
%f2ph = y(:,yi_fov);
%f1nh = inv(K)*f1ph;
%f2nh = inv(K)*f2ph;
% remove any features not present in both images (maintain correspondences)
Qidx = xi_fov & yi_fov;
f1ph = x(:, Qidx);
f2ph = y(:, Qidx);
valid1 = xi_fov;
valid2 = yi_fov;

% Display summary
%------------------------------------------------
if (opt.debug == 1)
  disp(sprintf('--Stereo simulator'));
  disp(sprintf('Number of features: %d',opt.N));
  disp(sprintf('Image scale factor: %f',opt.imscale));
  if (opt.do_quantization == 1)
    disp(sprintf('Pixel quantization: ON'));
  else
    disp(sprintf('Pixel quantization: OFF'));
  end
  if (opt.do_f2_perturbation == 1)
    disp(sprintf('Feature perturbation: ON'));
    disp(sprintf('Feature perturbation (m,s): (%f,%f)',mean(opt.f2_mean),mean(opt.f2_std)));
  else
    disp(sprintf('Feature perturbation: OFF'));
  end
  if (opt.do_epipolar == 1)
      disp(sprintf('Epipolar orthogonal projection (m,v): (%e,%e)',mean_op,std_op));
      %disp(sprintf('Mean triangulation residual: %e', mean(X_err)));
      %disp(sprintf('Mean triangulation error: %e', mean(QX_err)));
  end
  disp(sprintf('Focus of Expansion (FOE) (1): [%f %f]', e1ph));
  disp(sprintf('Focus of Expansion (FOE) (2): [%f %f]', e2ph));
  drawnow;
end
if (opt.do_plot == 1)
  drawnow;
end

