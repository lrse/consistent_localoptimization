%   SLAM Toolbox for MATLAB, created and maintained by
%
%   Copyright 2008, 2009, 2010 Joan Sola @ LAAS-CNRS.
%   Copyright 2011, 2012, 2013 Joan Sola.
%   Copyright 2015-     Joan Sola @ IRI-UPC-CSIC.
%   Programmers (for parts of the toolbox):
%   Copyright David Marquez and Jean-Marie Codol @ LAAS-CNRS
%   Copyright Teresa Vidal-Calleja @ ACFR.
%   See COPYING.TXT for full copyright license.
%
%   Implementation of the Local Optimization with Virtual Priors approach by
%
%   Copyright 2021 Gastón Castro and Facundo Pessacg

function Fac = makeFramePriorFactor(Frm,Fac,id)

% Unary absolute frame prior factor, same as makeAbsFactor but uses 
% the provided id and it doesn't register the new factor on the Frm

Fac.used   = true;  % Factor is being used ?
Fac.id     = id; % Factor unique ID

Fac.type   = 'absolute'; % {'motion','measurement','absolute'}
Fac.rob    = Frm.rob;
Fac.sen    = [];    % sen index
Fac.lmk    = [];    % lmk index
Fac.frames = Frm.frm;

% Ranges
Fac.state.r1 = Frm.state.r;
Fac.state.r2 = [];

% Go to minimal space, 7DoF --> 6DoF
[e, V_x] = qpose2vpose(Frm.state.x);
V        = V_x * Frm.state.P * V_x';

% Measurement is the straight data
Fac.meas.y    = Frm.state.x;
Fac.meas.R    = Frm.state.P;
% Fac.meas.W = [];  % measurement information matrix

% Expectation has zero covariance -- and info is not defined
Fac.exp.e     = Fac.meas.y;                 % expectation
Fac.exp.E     = zeros(size(Fac.meas.R));    % expectation cov
%     Fac.exp.W = Fac.meas.W; % expectation information matrix

% Error is zero at this stage, and takes covariance and info from measurement
Fac.err.z     = zeros(size(e)); % error or innovation (we call it error because we are on graph SLAM)
Fac.err.Z     = V;              % error cov matrix
Fac.err.W     = V^-1;           % error information matrix
Fac.err.Wsqrt = chol(Fac.err.W);

% Jacobians are zero at this stage. Just make size correct.
Fac.err.J1    = zeros(6,Frm.state.size); % Jac. of error wrt. node 1

Fac.err.size  = numel(e);

end