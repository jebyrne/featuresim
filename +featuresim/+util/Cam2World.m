function [TW1,TW2] = Cam2World(EO)
%--------------------------------------------------------------------
%
% File: Cam2World.m
%
% Copyright (c) 2013 Jeffrey Byrne <jebyrne@gmail.com>
%
%--------------------------------------------------------------------

% Convert Direction Cosine Matrix input to fixed Euler angles
R1 = Fixed2DCM(EO(1).W);
R2 = Fixed2DCM(EO(2).W);

% Camera frame transform: [X(right), Y(forward), Z(down)]
CR1 = R1;
CR2 = R2;
CR1(:,3) = -CR1(:,3);
CR2(:,3) = -CR2(:,3);
CT1 = [EO(1).T(1) EO(1).T(2) -EO(1).T(3)]';
CT2 = [EO(2).T(1) EO(2).T(2) -EO(2).T(3)]';

% Camera 2 written in camera frame 1
R12 = CR1'*CR2;            % DCM matrix  
T12 = (CR1'*(EO(2).T-EO(1).T));    % Translation (m)
W12 = DCM2Fixed(R12);      % Fixed Euler angles (rad)
if (norm(T12) ~= 0)
  H12 = T12 ./ norm(T12);    % Heading |H12|=1
else
  H12 = zeros(3,1);
end
  
% Camera 1 written in camera frame 2
R21 = R12';                % DCM matrix
W21 = DCM2Fixed(R21);      % Fixed Euler angles (rad)
T21 = -R12'*T12;           % Translation (m)
if (norm(T21) ~= 0)
  H21 = T21 ./ norm(T21);    % Heading 
else
  H21 = zeros(3,1);
end

% Camera projection matrices
%----------------------------------------------------
% Camera 1 projection matrix
P1 = [CR1' -CR1'*EO(1).T];  
T1W = [P1; 0 0 0 1];
TW1 = inv(T1W);

% Camera 2 projection matrix
P2 = [CR2' -CR2'*EO(2).T];
T2W = [P2; 0 0 0 1];
TW2 = inv(T2W);
