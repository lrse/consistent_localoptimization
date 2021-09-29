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

function [covFrames, visLmks] = relatedVariables(Fac,Frm,Lmk,targetFrame)
    covIndex = 1;
    visIndex = 1;
    
    covFrames.frame = [];
    covFrames.factors = [];
    
    visLmks.lmk = [];
    visLmks.factor = [];
    
    % Defining covisible area
    for fac = targetFrame.factors
        switch Fac(fac).type
            case 'motion'
                for facFrame = Fac(fac).frames
                    % Skip target frame
                    if facFrame == targetFrame.frm
                        continue
                    end
                    
                    ind = find([covFrames.frame] == facFrame, 1);
                    if isempty(ind)
                        covFrames(covIndex).frame = facFrame;
                        ind = covIndex;
                        covIndex = covIndex+1;
                    end
                    covFrames(ind).factors = [covFrames(ind).factors fac];
                end
            case 'measurement'
                lmk = Fac(fac).lmk;
                
                visLmks(visIndex).lmk = lmk;
                visLmks(visIndex).factor = fac;
                visIndex = visIndex+1;
                
                for lmkFac = Lmk(lmk).factors
                    for facFrame = Fac(lmkFac).frames
                        % Skip target frame
                        if facFrame == targetFrame.frm
                            continue
                        end
                        
                        ind = find([covFrames.frame] == facFrame, 1);
                        if isempty(ind)
                            covFrames(covIndex).frame = facFrame;
                            ind = covIndex;
                            covIndex = covIndex+1;
                        end
                        covFrames(ind).factors = [covFrames(ind).factors fac];                        
                    end
                end
        end
    end
end