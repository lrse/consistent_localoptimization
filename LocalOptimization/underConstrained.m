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

function [underConstrainedFrms, underConstrainedLmks] = underConstrained(Frm,Lmk,Fac,rob,focusedFrames,focusedLmks,localFac,facIndex)
    covIndex = 1;
    visIndex = 1;
    
    underConstrainedFrms.frames = [];
    underConstrainedFrms.factors = [];
    
    underConstrainedLmks.lmks = [];
    underConstrainedLmks.factors = [];
    
    % Computing under constrained frames
    for focFrm = focusedFrames
        focFrame = Frm(rob,focFrm);
        totalFac = focFrame.factors;
        % NOTE: Using arrayfun as Fac(i).frames could have some [] fields and that breaks finds's results
        includedLocalFac = find(arrayfun(@(s) ismember(focFrm, s.frames), localFac(1:facIndex)));
        includedFac = ismember(totalFac, [localFac(includedLocalFac).fac]);
        if ~all(includedFac)
            % Adding frame as underconstrained
            underConstrainedFrms.frames = [underConstrainedFrms.frames focFrm];
            % Adding factors that where not included
            underConstrainedFrms.factors = [underConstrainedFrms.factors totalFac(~includedFac)];
        end
    end

    % Computing under constrained landmarks
    if ~isempty(focusedLmks)
        for foclmk = focusedLmks
            totalFac = Lmk(foclmk).factors;
            % NOTE: Using arrayfun as Fac(i).lmk could have some [] fields and that breaks finds's results
            includedLocalFac = find(arrayfun(@(s) ismember(foclmk, s.lmk), localFac(1:facIndex)));
            includedFac = ismember(totalFac, [localFac(includedLocalFac).fac]);
            if ~all(includedFac)
                % Adding lmk as underconstrained
                underConstrainedLmks.lmks = [underConstrainedLmks.lmks foclmk];
                % Adding factors that where not included
                underConstrainedLmks.factors = [underConstrainedLmks.factors totalFac(~includedFac)];
            end
        end
    end
end