function [opt_out] = get_options(opt_in, opt_default, opt_error_flag)
%--------------------------------------------------------------------
%
% File: get_options.m
% Authors: Jeffrey Byrne (jbyrne@ssci.com)
%
% Description:  Copy default options to output, overwriting those
% options provided as input.  If an option structure field is not
% found, then issue warning or error as determined by optional third
% argument. 
%
% Inputs:
%   opt_in: input options structure 
%   opt_default: default options structure.  
%   opt_error_flag:
%      0 (default) : issue warning if option is not among defaults.
%      1 : raise error if option is not among defaults.
%     -1 : allow options not specified in defaults.
%
% Outputs:
%   opt_out: output options structure
%
% Copyright (c) 2013 Jeffrey Byrne <jebyrne@gmail.com>
%
%--------------------------------------------------------------------

% Input check
if (nargin < 2) || (nargin > 3)
  error('Invalid input');
end
if nargin<3
  opt_error_flag = 0; % issue warning if an option is not found in defaults
end

% Set options: Overwrite defaults with provided options
opt_out = opt_default;
if (isempty(opt_in) == 0)
  optin_fields = fieldnames(opt_in);
  for f=1:length(optin_fields)
    field_name = optin_fields{f}; % char(optin_fields(f));
    % Match field
    matched_option = 0;
    if (isfield(opt_out,field_name) == 1)
      matched_option = 1;
    end
    if (matched_option==0)
      if (opt_error_flag==0)
        warning('get_options:InvalidOption', 'Invalid option field: ''%s''', field_name)
      elseif (opt_error_flag==1)
        error('get_options:InvalidOption', 'Invalid option field: ''%s''', field_name);
      end
    end
    if (matched_option==1 || opt_error_flag~=1)
      %opt_out = setfield(opt_out,field_name,getfield(opt_in,field_name));
      opt_out.(field_name) = opt_in.(field_name);
    end
  end
end
