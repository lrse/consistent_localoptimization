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

function [Fac] = makePropagatedRelativeMotionFactor(Frm_old, Frm_new, Fac, indexFac)

%   Creates a motion factor Fac from the past frame Frm_old to the
%   frame Frm_new, using the Frm_new covariance.

global Map

Fac.used = true; % Factor is being used ?
Fac.id = indexFac; % Factor unique ID

Fac.type = 'motion'; % {'motion','measurement','absolute'}
Fac.rob = Frm_old.rob;
Fac.sen = [];
Fac.lmk = [];
Fac.frames = [Frm_old.frm Frm_new.frm];

% Ranges
Fac.state.r1 = Frm_old.state.r;
Fac.state.r2 = Frm_new.state.r;

[x1_x2_incr, J_x1, J_x2] = frameIncrement(...
    Frm_old.state.x, ...
    Frm_new.state.x);

% Poses covariance with zero correlation
%E = [Frm_new.state.P zeros(size(Frm_new.state.P)) ; zeros(size(Frm_old.state.P)) Frm_old.state.P];

% Pose covariances with the correlation that we have until this moment.
% Might not be complete and E could break semi-definite positive
% In non minimal space, 6DoF --> 7DoF, to operate with increment jacobians
% Enforcing semi-definite positiveness using Nicholas Higham's math
M = [Frm_old.state.M zeros(size(Frm_old.state.M)) ; zeros(size(Frm_new.state.M)) Frm_new.state.M];
E = M * nearestSPD(full(Map.P([Frm_old.state.r Frm_new.state.r], [Frm_old.state.r Frm_new.state.r]))) * M';

prop_cov = [J_x1 J_x2] * E * [J_x1 J_x2]';

% Measurement and his uncertainty is expected to be in quat space (see computeError.m)
Fac.meas.y = x1_x2_incr;
Fac.meas.R = prop_cov;
% Fac.meas.W = []; % measurement information matrix

% Expectation has zero covariance -- and info is not defined 
Fac.exp.e = Fac.meas.y; % expectation (will be recomputed in computeError.m)
Fac.exp.E = zeros(size(Fac.meas.R)); % expectation cov
%     Fac.exp.W = Fac.meas.W; % expectation information matrix

% Uncertainty of the error is expected in euler angle space (see computeError.m)
[e, V_x] = qpose2vpose(x1_x2_incr);
V = V_x * prop_cov * V_x';

% Error is zero at this stage, and takes covariance and info from measurement
Fac.err.z     = zeros(size(e)); % error or innovation (we call it error because we are on graph SLAM)
Fac.err.Z     = V;              % error cov matrix
Fac.err.W     = V^-1;           % error information matrix
Fac.err.Wsqrt = chol(Fac.err.W);

% Jacobians are zero at this stage. Just make size correct.
Fac.err.J1 = zeros(6,Frm_old.state.size); % Jac. of error wrt. node 1
Fac.err.J2 = zeros(6,Frm_new.state.size); % Jac. of error wrt. node 2

Fac.err.size  = numel(e);

end
