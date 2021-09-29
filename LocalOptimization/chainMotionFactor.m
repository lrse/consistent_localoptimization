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

function [localFac,usedIndices] = chainMotionFactor(oriFrame,localFac,facIndex,toMarginalize,usedIndices,Frm,rob)

    % Adding prior factor to origin
    localFac(facIndex) = makeFramePriorFactor(...
        oriFrame, ...
        localFac(facIndex), ...
        facIndex);
    facIndex = facIndex+1;
    usedIndices(oriFrame.state.r) = 1;        

    if ~isempty(toMarginalize)
        % If only one frame is left as to marginalize then we only
        % add it as variable, his motion factor is already added
        totalNodes = length(toMarginalize);
        if totalNodes == 1
            mFrm = Frm(rob, toMarginalize(1));
            usedIndices(mFrm.state.r) = 1;
        else
            % Sorting frames to be marginalized by covariance volume
            volumes = covarianceVolumes(Frm(rob, toMarginalize));
            % Sorting covariances by volume
            [sortCov, sortInd] = sort(volumes);
            prevInd = sortInd(1);

            % Connect frames from lower covariance to higher covariance
            for ind = sortInd(2:end)
                % Source frame
                srcfrm = toMarginalize(prevInd);
                srcFrm = Frm(rob, srcfrm);
                % Target frame
                trgfrm = toMarginalize(ind);
                trgFrm = Frm(rob, trgfrm);

                % Adding relative virtual motion factor
                localFac(facIndex) = makePropagatedRelativeMotionFactor(...
                    srcFrm, ...
                    trgFrm, ...
                    localFac(facIndex), ...
                    facIndex);
                facIndex = facIndex+1;

                usedIndices(srcFrm.state.r) = 1;
                usedIndices(trgFrm.state.r) = 1;

                prevInd = ind;
            end
        end
    end
end
