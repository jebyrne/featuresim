function [] = set_paths()
%--------------------------------------------------------------------------
%
% Copyright (c) 2013 Jeffrey Byrne <jebyrne@gmail.com>
%
%--------------------------------------------------------------------------


%% Disable name conflicts during
warning('off','MATLAB:dispatcher:nameConflict');  


%% Support paths
addpath(pwd);
addpath(fullfile(pwd),'test')


%% Unpack Dependencies
% none

%% Compiling
% none

%% Restore
warning('on','MATLAB:dispatcher:nameConflict');  

