%
% Expansion of sixth order polynomial for optimal triangulation
% Refer to Hartley+Zisserman p317, equation (12.7)
%
% Copyright (c) 2013 Jeffrey Byrne <jebyrne@gmail.com>
%
syms a b c d f fp g t
g = t*((a*t + b)^2 + (fp^2)*(c*t+d)^2)^2 - (a*d-b*c)*((1+(f^2)*(t^2))^2)*(a*t+b)*(c*t+d)
h = expand(g)
collect(h)
simple(h)
