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

function [Rob,Sen,Lmk,Obs,Frm,Fac] = solveLocalGraph(Rob,Sen,Lmk,Obs,Frm,Fac,Opt)

% SOLVELOCALGRAPH Solves the Local SLAM graph using Cholesky decomposition.
%   This implementation is based on solveGraphCholesky with extensions to
%   support variables marginalization applying schur complement.
%
%   Matrix re-ordering has been erased to simplify operations and indices
%   bookkeeping. This should slow down a bit the linear system solving.
%
%   See also SOLVEGRAPHCHOLESKY.


global Map

% Control of iterations and exit conditions
n_iter      = Opt.solver.niterations; % exit criterion of number of iterations
target_dres = Opt.solver.target_dres; % exit criterion for error variation
target_res  = Opt.solver.target_res;  % exit criterion for current residual
res_old     = 1e10;                   % last iteration's error

% Map states range
Map.mr = find(Map.used);

% Is there any variable set to be 'schured-out'?
if isfield(Map,'toSchur')
    i = ismember(Map.mr, Map.toSchur);
    toSchur_mr = Map.mr(i);             % to be schured-out
    toSchur_complement_mr = Map.mr(~i); % to be kept
end

for it = 1:n_iter
    
%     fprintf('----------------\nIteration: %d; \n',it)

    
    % Compute Jacobians for projection onto the manifold
    [Frm,Lmk] = errorStateJacobians(Frm,Lmk);
    
    % Build Hessian H and rhs vector b, in global Map
    Fac       = buildProblem(Rob,Sen,Lmk,Obs,Frm,Fac);
    
    
    if isfield(Map,'toSchur') && ~isempty(toSchur_mr)
        % 'schuring-out' variables
        
        % Using moore-penrose pseudoinverse, in case of non-invertible
        iH = pinv(full(Map.H(toSchur_mr,toSchur_mr))); 
        
        % Getting Schur complement sH
        sH = Map.H(toSchur_complement_mr,toSchur_complement_mr) - Map.H(toSchur_complement_mr,toSchur_mr) * iH * Map.H(toSchur_mr, toSchur_complement_mr);
        sb = Map.b(toSchur_complement_mr) - Map.H(toSchur_complement_mr,toSchur_mr) * iH * Map.b(toSchur_mr);
        
        % Decomposition
        [Map.R, ill] = chol(sH);
        
        if ill
            warning('Ill-conditioned Hessian')
        end
        
        % Solve for dx, only for the 'to be kept' variables:
        %   - dx is Map.x(toSchur_complement_mr)
        y         = -Map.R'\sb; % solve for y
        Map.x(toSchur_complement_mr) = Map.R\y;
    else
        % Decomposition
        [Map.R, ill] = chol(Map.H(Map.mr,Map.mr));
        
        if ill
            warning('Ill-conditioned Hessian')
        end
        
        % Solve for dx:
        %   - dx is Map.x(mr)
        y         = -Map.R'\Map.b(Map.mr); % solve for y
        Map.x(Map.mr) = Map.R\y;
    end
    
    % Update nominal states
    [Rob,Lmk,Frm] = updateStates(Rob,Lmk,Frm);
    
    % Check resulting errors
    [res, err_max] = computeResidual(Rob,Sen,Lmk,Obs,Frm,Fac);
    dres           = res - res_old;
    res_old        = res;
    
    %         fprintf('Residual: %.2e; variation: %.2e \n', res, dres)
    
    if ( ( -dres <= target_dres ) || (err_max <= target_res) ) %&& ( abs(derr) < target_derr) )
        break;
    end
    
    % NOTE: THIS CODE UPDATES THE COVARIANCE WITHIN THE OPTIMIZATION
    % ITERATION, THIS WOULD AFFECT PRIORS ERROR COMPUTATION WITHIN EVERY ITERATION.
%     % Permutation matrix applied to H
%     P = eye(max(pr)); % max is taken istead of length as there could be a diference between Map.used and Fac.used
%     P = P(pr, pr);
% 
%     iR = inv(Map.R);
%     Map.P(pr, pr) = P * iR * iR' * P';
%     
%     % re-linearize manifold jacobians
%     [Frm,Lmk] = errorStateJacobians(Frm,Lmk);
%     % Update covariances
%     [Rob,Lmk,Frm] = updateCovariances(Rob,Lmk,Frm);
    
end

% This is flag-protected above, kept only as reference
if(size(Map.R,1) ~= size(Map.R,2)) % If Cholesky is ill, use pseudoinverse
    iR = pinv(full(Map.R));
else
    iR = inv(Map.R);
end

% If there where variables 'schured-out', update only the ones to be kept
if isfield(Map,'toSchur') && ~isempty(toSchur_mr)
    Map.P(toSchur_complement_mr, toSchur_complement_mr) = iR * iR';
else
    Map.P(Map.mr, Map.mr) = iR * iR';
end

% re-linearize manifold jacobians
[Frm,Lmk] = errorStateJacobians(Frm,Lmk);
% Update covariances
[Rob,Lmk,Frm] = updateCovariances(Rob,Lmk,Frm);

end

function Fac = buildProblem(Rob,Sen,Lmk,Obs,Frm,Fac)

% BUILDPROBLEM Build least squares problem's matrix H and vector b 
%   Fac = BUILDPROBLEM(Rob,Sen,Lmk,Obs,Frm,Fac) Builds the least squares
%   problem's matrix H and vector b for a solution using sparse Cholesky
%   factorization of H.

global Map

% Reset Hessian and rhs vector
Map.H(Map.mr,Map.mr) = 0;
Map.b(Map.mr)    = 0;

% Iterate all factors
for fac = find([Fac.used])
    
    % Extract some pointers
    rob    = Fac(fac).rob;
    sen    = Fac(fac).sen;
    lmk    = Fac(fac).lmk;
    frames = Fac(fac).frames;
    
    % Compute factor error, info mat, and Jacobians
    [Fac(fac), e, W, ~, J1, J2, r1, r2] = computeError(...
        Rob(rob),       ...
        Sen(sen),       ...
        Lmk(lmk),       ...
        Obs(sen,lmk),   ...
        Frm(frames),    ...
        Fac(fac));

    % Compute sparse Hessian blocks
    H_11 = J1' * W * J1;
    H_12 = J1' * W * J2;
    H_22 = J2' * W * J2;
    
    % Compute rhs vector blocks
    b1   = J1' * W * e;
    b2   = J2' * W * e;
    
    % Update H and b
    Map.H(r1,r1) = Map.H(r1,r1) + H_11;
    Map.H(r1,r2) = Map.H(r1,r2) + H_12;
    Map.H(r2,r1) = Map.H(r2,r1) + H_12';
    Map.H(r2,r2) = Map.H(r2,r2) + H_22;

    Map.b(r1,1)  = Map.b(r1,1)  + b1;
    Map.b(r2,1)  = Map.b(r2,1)  + b2;

end

end




% ========== End of function - Start GPL license ==========


%   # START GPL LICENSE

%---------------------------------------------------------------------
%
%   This file is part of SLAMTB, a SLAM toolbox for Matlab.
%
%   SLAMTB is free software: you can redistribute it and/or modify
%   it under the terms of the GNU General Public License as published by
%   the Free Software Foundation, either version 3 of the License, or
%   (at your option) any later version.
%
%   SLAMTB is distributed in the hope that it will be useful,
%   but WITHOUT ANY WARRANTY; without even the implied warranty of
%   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%   GNU General Public License for more details.
%
%   You should have received a copy of the GNU General Public License
%   along with SLAMTB.  If not, see <http://www.gnu.org/licenses/>.
%
%---------------------------------------------------------------------

%   SLAMTB is Copyright:
%   Copyright (c) 2008-2010, Joan Sola @ LAAS-CNRS,
%   Copyright (c) 2010-2013, Joan Sola,
%   Copyright (c) 2014-2015, Joan Sola @ IRI-UPC-CSIC,
%   SLAMTB is Copyright 2009
%   by Joan Sola, Teresa Vidal-Calleja, David Marquez and Jean Marie Codol
%   @ LAAS-CNRS.
%   See on top of this file for its particular copyright.

%   # END GPL LICENSE

