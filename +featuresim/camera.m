function [fph,f_valid,K,P_enu2cam,Q] = camera(x_bff_in_ned, x_ned_in_enu, x_cam_in_bff, Q, optin)
%--------------------------------------------------------------------
%
% File: featuresum.camera
% Author: Jeffrey Byrne (jebyrne@gmail.com)
%
% Description:  Given a set of 3D points in the world, one camera with 
% known exterior orientation and intrinsics, project the 3D points 
% to the image.  
%
% Inputs: All poses x=[X,Y,Z,Roll,Pitch,Yaw]^T in meters/radians
%   x_bff_in_ned: pose of body fixed frame 1 in north east down (NED)
%   x_ned_in_enu: pose of NED in east north up (ENU) world frame
%   x_cam_in_bff (optional): pose of camera base in body fixed frame
%     Note that camera base poses are subsequently mapped by bff2cam
%     from camera base frame XYZ axes to actual camera ZXY axes; e.g. given
%     default x_cam_in_bff euler angles of [0 -pi/2 0], the camera X axis
%     is aligned with body Y (right wing), camera Y is aligned with body -X
%     (out tail), and camera/body Z axes are aligned (boresight downward).
%   Q (optional): 4xN homogeneous matrix of 3D points in ENU
%   optin: options structure (see test_stereo_simulator.m)
%
% Outputs:
%   fph: 3xN features (f) in image, pixel coordinates (p), dehomogenized (h)
%   f_valid: 1xN indicator vector for features within field of view
%   K: 3x3 intrinsic calibration matrix
%   P_enu2cam: 3x4 camera projection matrix for points in ENU to image
%   Q: 4xN 3D features in ENU (provided if Q input is empty)
%
% Frames:
%   CAM: X (right), Y (down), Z (optical axis, out).
%   BFF: X (out nose), Y (right), Z (down)
%   NED: X (out nos, north), Y (right, east), Z (down) (BFF attitude reference)
%   ENU: X (east), Y (north), Z (up).  Mapping/Global frame
% 
% Copyright (c) 2006 Jeffrey Byrne <jebyrne@gmail.com>
%
%--------------------------------------------------------------------

%% Input check
if ~exist('Q','var')
  Q = [];
end
if ~exist('x_cam_in_bff','var')
  % Point camera base X along body Z axis, aligning camera & body Z axes
  x_cam_in_bff = [0 0 0 0 -pi/2 0]'; % (Downward looking)
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


%% Options
default_opt = featuresim.opts();

% Set options: Overwrite defaults with provided options
opt = featuresim.util.get_options(optin, default_opt, -1);


%% Intrinsic camera parameters
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


%% Camera pose: Exterior orientation
% Randomly generate exterior orientation (x) if requested
if (opt.do_eo == 1)
  x_bff_in_ned = opt.eo_t_mean + opt.eo_t_std.*randn(6,1);
end
[P_enu2cam] = featuresim.util.bff2cam(x_bff_in_ned, x_ned_in_enu, x_cam_in_bff);

% Origin of camera 1 in ENU
T_enu2cam = [P_enu2cam; 0 0 0 1];
T_cam2enu = T_enu2cam\eye(4);
o_cam_in_enu = T_cam2enu*[0 0 0 1]';
o_cam_in_enu_h = o_cam_in_enu(1:3) ./ o_cam_in_enu(4);


%% Generate 3D points in ENU
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

   case 'biplanargrid'    
    % Generate random 3D points on 2 planes    
    [X,Y] = meshgrid(-1:1/opt.N:1,-1:1/opt.N:1);
    k = find(sqrt((X(:).^2+Y(:).^2)) < 0.5);
    N = length(X(:));
    Z = ones(1,N);
    Z(k) = 2;
    Q = [X(:)'; Y(:)'; Z; ones(1,N)];   
    
   case 'rectgrid'
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

   case 'squaregrid'
    % Grid points: (m)
    Q = [-1 -1 0 1;
         -1 0 0 1;
         -1 1 0 1;
         0 -1 0 1;
         0 0 0 1;     
         0 1 0 1;          
         1 -1 0 1;
         1 0 0 1;     
         1 1 0 1]';
    opt.N = size(Q,2);
    Q(1:2,:) = 1*Q(1:2,:);
    Q(3,:) = zeros(1,opt.N);
    
   otherwise
    error('Undefined pointcloud mode ''%s''',opt.pointcloud_mode);
  end
else
  opt.N = size(Q,2);
end



%% Perspective projection
% Project to image 1 in image coordinates
x = K*P_enu2cam*Q;
xi_pos = x(3,:)>0; % flag 'valid' points (in front of camera)
x = x ./ repmat(x(3,:),3,1);

% Quantization scale factor
qscale = 1;
if isfield(opt,'quantization_factor'),
  qscale = opt.quantization_factor;
end

% Feature perturbation
x_true = x;
if (opt.do_perturbation == 1)
  dx = opt.f_mean + opt.f_std.*randn(size(x(1:2,:)));
  x(1:2,:) = x(1:2,:) + dx;
end

% Round to nearest pixel
x_unquantized = x;
if (opt.do_quantization == 1)
  x = round(x/qscale)*qscale;
  %x = round(x);
end


% Enforce field of view
xi_fov = (x(1,:) >= 1) & (x(1,:) <= W) & (x(2,:) >= 1) & (x(2,:) <= H);

% Apply positive depth constraints (points behind camera are not visible)
xi_fov = xi_fov & xi_pos;


%% Feature outputs
% remove any features not present in both images (maintain correspondences)
Qidx = xi_fov;
fph = x(:, Qidx);
f_valid = xi_fov;


%% Summary
if opt.debug
  disp(sprintf('--Camera simulator'));
  disp(sprintf('Number of features: %d',opt.N));
  disp(sprintf('Image scale factor: %f',opt.imscale));
  if (opt.do_quantization == 1)
    disp(sprintf('Pixel quantization: ON'));
  else
    disp(sprintf('Pixel quantization: OFF'));
  end
  if (opt.do_perturbation == 1)
    disp(sprintf('Feature perturbation: ON'));
    disp(sprintf('Feature perturbation variance: %f',opt.f_std));
  else
    disp(sprintf('Feature perturbation: OFF'));
  end  
  disp(sprintf('--Camera poses'));  
  fprintf('Camera frame (CAM) in body fixed frame (BFF) of vehicle: (%1.3fm, %1.3fm, %1.3fm, %1.3fr, %1.3fr, %1.3fr) (XYZRPY)\n', x_cam_in_bff)
  fprintf('Body fixed frame (BFF) in north east down (NED) inertial frame: (%1.3fm, %1.3fm, %1.3fm, %1.3fr, %1.3fr, %1.3fr) (XYZRPY)\n', x_bff_in_ned)
  fprintf('North east down (NED) inertial frame in east north up (ENU) mapping frame: (%1.3fm, %1.3fm, %1.3fm, %1.3fr, %1.3fr, %1.3fr) (XYZRPY)\n', x_ned_in_enu)
end


%% Display 
% Image projections are in image coordinates (u,v)=(right,down)
% Origin: (u0,v0)=upper left
if (opt.do_plot == 1)
  h = figure(sum(char(mfilename))); clf;
  set(h,'NumberTitle','off','Name','FeatureSim');

  subplot(1,2,1); hold on; title('Image');   
  axis equal; axis([1 W 1 H]); grid on;
    
  % Convert plot to image coordinates
  axis ij; set(gca,'XAxisLocation','top');
   
  % 3D point cloud
  subplot(1,2,2); hold on; title('3D Point Cloud');   
  grid on;

  % Point cloud
  subplot(1,2,2); plot3(Q(1,:),Q(2,:),Q(3,:),'g.'); hold on;
  plot3(0,0,0,'bo');
  xlabel('X (m)');ylabel('Y (m)');zlabel('Z (m)');
  
  % Display camera centers of projection
  plot3(o_cam_in_enu_h(1),o_cam_in_enu_h(2),o_cam_in_enu_h(3),'bs','MarkerSize',20);
  text(o_cam_in_enu_h(1),o_cam_in_enu_h(2),o_cam_in_enu_h(3),'1','Color',[0 0 1]);

  % Plot massage
  view(3); axis equal;  
  hold off;

  % Projection
  subplot(1,2,1); plot(x_true(1,:), x_true(2,:), 'g.'); box on; 
  subplot(1,2,1); plot(x_unquantized(1,:), x_unquantized(2,:), 'b+');
  subplot(1,2,1); plot(x(1,:), x(2,:), 'r.');    

end


