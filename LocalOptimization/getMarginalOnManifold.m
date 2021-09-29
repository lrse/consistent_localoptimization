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

function [Mdiag, Mcross, variablesRs] = getMarginalOnManifold(Rob,Lmk,Frm,Fac,Sen,Obs,localFac)
    global Map
    
    % Covariances marginal of involved variables
    Mdiag = sparse([]);
    Mcross = sparse([]);
    
    % Indices of involved variables (to be applied on Map.H or Map.P)
    variablesRs = [];
    
    % Iterate all factors
    % Iteration order matches the one used in buildProblem
    for fac = localFac

        % Extract some pointers
        rob    = fac.rob;
        sen    = fac.sen;
        lmk    = fac.lmk;
        frames = fac.frames;
        
        for frm = frames
            % Current marginal size
            marginalSize = size(Mdiag, 1);
            
            % Taking current set of indices
            r = Frm(rob, frm).state.r;
            stateSize = Frm(rob, frm).state.size;
            manifStateSize = Frm(rob, frm).state.dsize;
            
            % Has already a marginal index assigned?
            if isfield(Frm(rob, frm).state, 'rm')
                rm = Frm(rob, frm).state.rm;
            else
                % Assigning index on marginal and adding his marginal
                rm = marginalSize+1:marginalSize+manifStateSize;
                Frm(rob, frm).state.rm = rm;
                
                % Adding diagonal block
                Mdiag(rm, rm) = Map.P(r, r);
                
                variablesRs = [variablesRs ; r];
            end
                
            % Adding cross marginal of related frames already added
            % Note: this way each new block is in charge of adding
            % cross block of already added variables
            for oldfrm = frames
                if frm ~= oldfrm && isfield(Frm(rob, oldfrm).state, 'rm')
                    oldr = Frm(rob, oldfrm).state.r;
                    oldrm = Frm(rob, oldfrm).state.rm;
                    Mcross(rm, oldrm) = Map.P(r, oldr);
                    Mcross(oldrm, rm) = Map.P(oldr, r);
                end
            end

            % Adding cross marginal of related landmarks already added
            if ~isempty(lmk) && isfield(Lmk(lmk).state, 'rm')
                oldr = Lmk(lmk).state.r;
                oldrm = Lmk(lmk).state.rm;
                Mcross(rm, oldrm) = Map.P(r, oldr);
                Mcross(oldrm, rm) = Map.P(oldr, r);
            end
        end
        
        % Adding landmark that where not added before
        if ~isempty(lmk) && ~isfield(Lmk(lmk).state, 'rm')
            % Current marginal size
            marginalSize = size(Mdiag, 1);
            
            % Taking current set of indices
            r = Lmk(lmk).state.r;
            stateSize = Lmk(lmk).state.size;
            manifStateSize = Lmk(lmk).state.dsize;
            
            % Assigning index on marginal and adding his marginal
            rm = marginalSize+1:marginalSize+manifStateSize;
            Lmk(lmk).state.rm = rm;

            % Adding diagonal block
            Mdiag(rm, rm) = Map.P(r, r);
            
            % Each new added variable is in charge of adding cross
            % marginals of already added variables
            for frm = frames
                % Has already a marginal index assigned?
                if isfield(Frm(rob, frm).state, 'rm')
                    oldr = Frm(rob, frm).state.r;
                    oldrm = Frm(rob, frm).state.rm;
                    Mcross(rm, oldrm) = Map.P(r, oldr);
                    Mcross(oldrm, rm) = Map.P(oldr, r);
                end
            end
            
            % TODO: by design, there are no factors between landmarks
            
            variablesRs = [variablesRs ; r];
        end
    end
end