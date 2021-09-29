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

function [localFac, usedIndices, Frm] = relativeTopologyFactorsLocalOptimization(Rob,Sen,Lmk,Obs,Frm,Fac,Trj,Opt)
    global Map
    
    % Factor pre-allocation
    localFac = createFactors(2000);
    
    usedIndices = Map.used;
    usedIndices(find(Map.used)) = 0; %reset used

    % Local factor index
    facIndex = 1;

    for rob = [Rob.rob]
        
        % Reset frames color
        for i = 1: size(Frm,2)
            Frm(rob,i).color = 0;
        end 
        
        % Reference keyframe as last one added
        reffrm = Trj(rob).head;
        refFrame = Frm(rob, reffrm);
        
        covFrames = [];
        visLmks = [];
        
        [covFrames, visLmks] = relatedVariables(Fac,Frm,Lmk,refFrame);
        
        % Adding reference keyframe and visible landmarks to local
        % optimization
        usedIndices(refFrame.state.r) = 1;
        
        % Adding landmarks seen by ref keyframe
        if ~isempty([visLmks.lmk]) % Checking if refframe do not sees any lmk
            for visLmk = visLmks
                usedIndices(Lmk(visLmk.lmk).state.r) = 1;
                localFac(facIndex) = Fac(visLmk.factor);
                facIndex = facIndex+1;
            end
        end
        
        % Focused area starts as last keyframe and all visible lmks
        focusedFrames = [reffrm];
        focusedLmks = [visLmks.lmk];
        
        % Storing covisibility degree on a separated array
        covDegrees = [];
        for covFrm = covFrames
            covDegrees = [covDegrees size(covFrm.factors,2)];
        end
        
        % Computed covariances volumes
        covarVolumes = covarianceVolumes(Frm(rob, [covFrames.frame]));
        
        % Local Optimization window threshold
        windowSize = Opt.map.localWindowSize -1;
        
        % Changes between 0 and 1 to prioritize different add policies
        oscilator = 0;
        allFrmAdded = false;
        
        while (windowSize > 0 && ~allFrmAdded) 
            % Select keyframes from the covisivily group
            if ~isempty([covFrames.frame]) 
                
                if oscilator == 0 % adds a max covisibility degree keyframe
                    [covDegree,ind] = max(covDegrees);
                    oscilator = oscilator+1;
                elseif oscilator == 1 % adds a min covariance volume keyframe
                    [covarianceVol,ind] = min(covarVolumes);
                    oscilator = oscilator+1;
                else
                    [covarianceVol,ind] = max(covarVolumes);
                    oscilator = 0;
                end

                % Selected covisible keyframe
                selectedCovFrame = covFrames(ind);
                maxFrame = Frm(rob, selectedCovFrame.frame);

                % Temp degree variable
                degree = covDegrees(ind);

                % Erasing keyframe candidate from arrays
                covFrames(ind) = [];
                covDegrees(ind) = [];
                covarVolumes(ind) = [];

                % Checking minimum degree of covisibility before adding it
                if degree < 2
                    continue;
                end               

                % Check if this is the last frame to add
                totalFrm = length(find([Frm.used]));
                if length(focusedFrames)+1 == totalFrm
                    allFrmAdded = true;
                end
                
            % If there are not more covisible keyframes, select the last keyframes according to the timeline (like a sliding window)
            else
                prevFrame = Frm(rob, refFrame.frm -1);
                frmAdded = false;
                skipFrame = false;
                while ~frmAdded 
                    
                    totalFrm = length(find([Frm.used]));
                    isFirstFrame = strcmp(Fac(prevFrame.factors(1)).type, 'absolute');
                    %if is the fisrt frame and it wasnot added yet, add it and finish 
                    if isFirstFrame && ~any(prevFrame.frm == focusedFrames)
                        allFrmAdded = true;
                        frmAdded = true;
                    % if it is the first frame and it was already added, skip it and finish
                    elseif isFirstFrame && any(prevFrame.frm == focusedFrames)
                        allFrmAdded = true;
                        skipFrame = true;
                        frmAdded = true;
                    % if is not the first frame and it wasnot added, add it 
                    elseif ~isFirstFrame && ~any(prevFrame.frm == focusedFrames)
                        frmAdded = true; 
                    else
                        % if is not the first frame and it was added, try the next one
                        prevFrame = Frm(rob, prevFrame.frm -1);
                    end
                end                              
                if skipFrame
                    continue;
                end 
                maxFrame = prevFrame;
            end 
             

            % Compute related keyframes and lmks
            [sndDegreeCovFrames, sndVisLmks] = relatedVariables(Fac,Frm,Lmk,maxFrame);

            % Adding keyframe and his visible landmarks
            usedIndices(maxFrame.state.r) = 1;
            focusedFrames = [focusedFrames maxFrame.frm];

            % Checking if covframe sees any lmk
            if ~isempty([sndVisLmks.lmk])
                for svLmk = sndVisLmks % Adding his visible landmarks
                    usedIndices(Lmk(svLmk.lmk).state.r) = 1;
                    localFac(facIndex) = Fac(svLmk.factor);
                    facIndex = facIndex+1;
                    focusedLmks = [focusedLmks svLmk.lmk];
                end
            end

            windowSize = windowSize-1;
        end
        
        % There is no checking while addition of lmks, so we erase
        % duplicates
        focusedLmks = unique(focusedLmks, 'stable');
        
        % Adding every motion factor between focused frames inside
        % optimization window
        for focFrm = focusedFrames
            focFrame = Frm(rob,focFrm);
            for fac = focFrame.factors
                if strcmp(Fac(fac).type,'motion')
                    % Motion factor between two focusedFrames
                    if Fac(fac).frames(1) == focFrm && ...
                       any(focusedFrames == Fac(fac).frames(2))
                        localFac(facIndex) = Fac(fac);
                        facIndex = facIndex+1;
                    end
               end
            end
        end
        
        [underConstrainedFrms, underConstrainedLmks] = underConstrained(Frm,Lmk,Fac,rob,focusedFrames,focusedLmks,localFac,facIndex);

        % Checking focus frames that do not have "motion in" factors
%         noMotionIn = [];
%         for uFac = underConstrainedFrms.factors
%             if strcmp(Fac(uFac).type,'motion')
%                 % Motion factor towards a focus that has not been added
%                 if any(focusedFrames == Fac(uFac).frames(2))
%                     noMotionIn = [noMotionIn Fac(uFac).frames(2)];
%                 end
%             end
%         end
        
        toMarginalize = [];
        
        % If an underconstrained lmk -> frame factor exists, then that
        % frame is not part of focusedFrms for the way that lmks are added
        for uFac = underConstrainedLmks.factors
            toMarginalize = [toMarginalize Fac(uFac).frames];
            localFac(facIndex) = Fac(uFac);
            facIndex = facIndex+1;
        end
        
        % If an underconstrained frame -> frame motion factor exists, then
        % we added it and set the 'ending' frame as to be marginalize
        for uFac = underConstrainedFrms.factors
            if strcmp(Fac(uFac).type,'motion')
                % Motion factor between two focusedFrames
                if any(focusedFrames == Fac(uFac).frames(1)) &&...
                   ~any(focusedFrames == Fac(uFac).frames(2))
                    toMarginalize = [toMarginalize Fac(uFac).frames(2)];
                    localFac(facIndex) = Fac(uFac);
                    facIndex = facIndex+1;
                end
                if ~any(focusedFrames == Fac(uFac).frames(1)) &&...
                   any(focusedFrames == Fac(uFac).frames(2))
                    toMarginalize = [toMarginalize Fac(uFac).frames(1)];
                    localFac(facIndex) = Fac(uFac);
                    facIndex = facIndex+1;
                end
            end
        end
        
        % Keyframes selected for marginalization
        toMarginalize = unique(toMarginalize, 'stable');
        
        % BUG: TEMPORAL FIX: There are cases where focusedFrames has
        % repetitions due to the policy of adding keyframes in temporal order.
        focusedFrames = unique(focusedFrames, 'stable');

        % Defining origin keyframe with prior factor
        joined = [focusedFrames(2:end) toMarginalize];
        volumes = covarianceVolumes(Frm(rob, joined));
        % Sorting covariances by volume
        [minCov, oriInd] = min(volumes);

        % Taking lowest covariance volume as prior origin
        origin = joined(oriInd);
        oriFrame = Frm(rob, origin);
        
        switch Opt.marginal.topology
            case 'chain'    
                [localFac,usedIndices] = chainMotionFactor(...
                    oriFrame,...
                    localFac,...
                    facIndex,...
                    toMarginalize,...
                    usedIndices,...
                    Frm,rob);
            
            case 'SchurOut'
                % Setting for 'schuring-out' every non-selected variable of
                % the graph
                
                % Restoring all factors to be considered
                localFac = Fac;
                
                % Setting all variables as to be 'schured-out' initially
                Map.toSchur = find(Map.used);
                % Only focusedFrames where flagged as locally used
                focused = ismember(Map.toSchur, find(usedIndices));
                % Excluding those selected, keeping them
                Map.toSchur(focused) = 0;
                
                % Restoring all variables to be considered
                usedIndices = Map.used;
        end

        % Change the color of the focused frames
        for i = focusedFrames
            Frm(rob,i).color = 1; 
        end 
        if ~isempty(toMarginalize)
            for i = toMarginalize
                Frm(rob,i).color = 2; 
            end 
        end
    end
end

