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

function [M, A, uM, uA] = getMarginalOnManifold(Rob,Lmk,Frm,Fac,Sen,Obs,targetVariable,focusedFrames,focusedLmks,focusedFac)
    global Map

    % Indices of involved variables (to be applied on Map.H or Map.P)
    variablesRs = [];
    % Current factor pointer on A matrix
    factorIndex = 1;
    
    % Matrix of involved Jacobians
    A = sparse([]);
    % Diagonal covariances marginals of involved variables
    Mdiag = sparse([]);
    
    % Indices of involved variables that remain unfocus (not included in focusedFac)
    variablesRs_u = [];
    % Current factor pointer on uA matrix
    factorIndex_u = 1;
    
    % Matrix of involved unfocused Jacobians
    uA = sparse([]);
    % Diagonal covariances marginals of involved unfocused variables
    Mdiag_u = sparse([]);
    
    for fac = targetVariable.factors
        % Extract some pointers
        rob    = Fac(fac).rob;
        sen    = Fac(fac).sen;
        lmk    = Fac(fac).lmk;
        frames = Fac(fac).frames;
        
        % Compute factor error, info mat, and Jacobians
        [Fac(fac), e, W, Wsqrt, J1, J2, r1, r2] = computeError(...
            Rob(rob),       ...
            Sen(sen),       ...
            Lmk(lmk),       ...
            Obs(sen,lmk),   ...
            Frm(frames),    ...
            Fac(fac));
        
        indices = cell(1,2);
        indices{1} = r1;
        indices{2} = r2;
        
        jacobians = cell(1,2);
        jacobians{1} = J1;
        jacobians{2} = J2;
        
        measureSize = size(W,1);
        
        for frm = frames
            % Current marginal size
            marginalSize = size(variablesRs,1);
            % Current unfocused marginal size
            marginalSize_u = size(variablesRs_u,1);
            
            % Taking current set of indices
            r = indices{1};
            
            stateSize = size(r,1);
            
            % Has already a marginal index assigned?
            if ~isfield(Frm(rob, frm).state, 'rm')
                % Assigning index on marginal and adding his marginal
                Frm(rob, frm).state.rm = marginalSize+1:marginalSize+stateSize;
                Mdiag(Frm(rob, frm).state.rm, Frm(rob, frm).state.rm) = Frm(rob, frm).state.M' * Frm(rob, frm).state.P * Frm(rob, frm).state.M;

                variablesRs = [variablesRs ; r];

                % It's unfocused?
                if ~any(focusedFrames == frm)
                    Frm(rob, frm).state.rm_u = marginalSize_u+1:marginalSize_u+stateSize;
                    Mdiag_u(Frm(rob, frm).state.rm_u, Frm(rob, frm).state.rm_u) = Frm(rob, frm).state.M' * Frm(rob, frm).state.P * Frm(rob, frm).state.M;

                    variablesRs_u = [variablesRs_u ; r];
                end
            end
            
            % Get current jacobian
            J = jacobians{1};
            jacobianSize = size(J,1);

            % Assign measure jacobian to jacobians matrix
            A(factorIndex:factorIndex+jacobianSize-1, Frm(rob, frm).state.rm) = Wsqrt * J;

            if ~any(focusedFrames == frm)
                uA(factorIndex_u:factorIndex_u+jacobianSize-1, Frm(rob, frm).state.rm_u) = Wsqrt * J;
            end

            % Drop jacobian and indices
            jacobians = jacobians(2:end);
            indices = indices(2:end);
        end
        
        if ~isempty(lmk)
            % Current marginal size
            marginalSize = size(variablesRs,1);
            % Current unfocused marginal size
            marginalSize_u = size(variablesRs_u,1);
            
            % Taking current set of indices
            r = indices{1};
            
            stateSize = size(r,1);

            if ~isfield(Lmk(lmk).state, 'rm')
                Lmk(lmk).state.rm = marginalSize+1:marginalSize+stateSize;
                Mdiag(Lmk(lmk).state.rm, Lmk(lmk).state.rm) = Lmk(lmk).state.P;

                variablesRs = [variablesRs ; r];
                
                % It's unfocused?
                if ~any(focusedLmks == lmk)
                    Lmk(lmk).state.rm_u = marginalSize_u+1:marginalSize_u+stateSize;
                    Mdiag_u(Lmk(lmk).state.rm_u, Lmk(lmk).state.rm_u) = Lmk(lmk).state.M' * Lmk(lmk).state.P * Lmk(lmk).state.M;

                    variablesRs_u = [variablesRs_u ; r];
                end
            end
            
            % Get current jacobian
            J = jacobians{1};
            jacobianSize = size(J,1);
            
            A(factorIndex:factorIndex+jacobianSize-1, Lmk(lmk).state.rm) = Wsqrt * J;
            
            if ~any(focusedLmks == lmk)
                uA(factorIndex_u:factorIndex_u+jacobianSize-1, Lmk(lmk).state.rm_u) = Wsqrt * J;
            end
            
            % Drop jacobian
            jacobians = jacobians(2:end);
            indices = indices(2:end);
        end
        
        factorIndex = factorIndex+measureSize;
        factorIndex_u = factorIndex_u+measureSize;
    end
    
    % TODO: SUPPORT CROSS MARGINALS
    
    %M = Map.P(variablesRs, variablesRs);
    M = Mdiag;
    
    %uM = Map.P(variablesRs_u, variablesRs_u);
    uM = Mdiag_u;
end