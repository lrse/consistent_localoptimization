function SenFig = createSenFig(Sen,Obs,SimLmk,FigOpt)

% CREATESENFIG  Create sensors figures and handles.
%   SENFIG = CREATESENFIG(Sen,Obs,SensorFigure) creates one sensor figure
%   per sensor, containing the following graphics objects:
%       - observed simulated landmarks (virtual world) - in red
%       - estimated landmarks observations, containing measurement and
%       uncertainty ellipsoid - in colors depending on the landmark type,
%       and a label with the landmark ID.
%
%   The output SENFIG is a structure array of handles to all these graphics
%   objects. See the Matlab documentation for information about graphics
%   handles and the way to efficiently manipulate graphics. SENFIG(sen) is
%   the structure of handles for sensor 'sen', with the following fields:
%       .fig     handle to the figure
%       .axes    handle to the axes
%       .raw     handle to raw perception, eg. the simulated landmarks
%       .measure array of handles to measurement points, one handle each
%       .ellipse array of handles to the ellipsoid's contour 'line' objects
%       .label   array of handles to the lmk's label 'text' objects
%
%   The figure is updated using drawSenFig.
%
%   See also DRAWSENFIG, CREATEMAPFIG, LINE, SET, GET.

%   Copyright 2008-2009 Joan Sola @ LAAS-CNRS.


% loop all sensors
for sen = 1:numel(Sen)
    
    % Figure
    if FigOpt.createVideo
        SenFig(sen).fig = figure(sen);
        set(SenFig(sen).fig, 'WindowStyle', 'normal');
        figPos     = get(SenFig(sen).fig,'position');
        figSize    = FigOpt.sensor.size;
        newFigPos       = [sen*figSize(1) figPos(2) figSize];
        set(SenFig(sen).fig,'position',newFigPos);
    else
        if ishandle(sen)
            % redefine figure handle
            SenFig(sen).fig = figure(sen);
        else
            % create new figure
            SenFig(sen).fig = figure(sen);
            if ~strcmp( get(0, 'DefaultFigureWindowStyle'), 'docked')
                figPos          = get(SenFig(sen).fig,'position');
                figSize         = FigOpt.sensor.size;
                newFigPos       = [sen*figSize(1) figPos(2) figSize];
                set(SenFig(sen).fig,'position',newFigPos);
            end
        end
    end
    moreindatatip   % this adds the data index in the figures data tips
    set(SenFig(sen).fig,...
        'numbertitle',  'off',...
        'name',         ['Robot ' num2str(Sen(sen).robot) '  --  Sensor ' num2str(sen) '  (' Sen(sen).type ')'],...
        'doublebuffer', 'on',...
        'color',        FigOpt.sensor.colors.border,...
        'renderer',     FigOpt.renderer);
    clf
    
    
    % Sensor type:
    % ------------
    switch Sen(sen).type
        
        % camera pinhole
        % --------------
        case {'pinHole','pinHoleDepth'}
            % axes
            axis equal
            SenFig(sen).axes = gca;
            set(SenFig(sen).axes,...
                'position',      [0 0 1 1],...                  %[.07 .05 .9 .9],...
                'color',         FigOpt.sensor.colors.bckgnd,...
                'box',           'on',...
                'xlim',          [0 Sen(sen).par.imSize(1)],...
                'ylim',          [0,Sen(sen).par.imSize(2)],... % size of the image of sensor
                'alimmode',      'manual',...
                'climmode',      'manual',...
                'xaxislocation', 'top',...
                'ydir',          'reverse',...
                'layer',         'top',...
                'fontsize',      9,...
                'xcolor',        FigOpt.sensor.colors.axes,...
                'ycolor',        FigOpt.sensor.colors.axes,...
                'zcolor',        FigOpt.sensor.colors.axes);
            
            
            % image
            SenFig(sen).img = image(...
                'cdata',[ ],...
                'alphadata',1,...
                'xdata',[1 Sen(sen).par.imSize(1)],...
                'ydata',[1 Sen(sen).par.imSize(2)]);
            
            colormap(gray(256));
            
            
            
            % raw points
            SenFig(sen).raw.points = line(...
                'parent',        SenFig(sen).axes,...
                'xdata',         [],...
                'ydata',         [],...
                'color',         FigOpt.sensor.colors.raw,...
                'linestyle',     'none',...
                'marker',        '+',...
                'markersize',    4);
            
            % raw segments
            if ~isempty(SimLmk)
                S = pinHoleSegment(Sen(sen).par.k,SimLmk.segments.coord);
                X = S([1 3],:);
                Y = S([2,4],:); % Take all segments to create a number of lines. There will be as many handles as segments.
                SenFig(sen).raw.segments = line(...
                    X,...
                    Y,...
                    'parent',        SenFig(sen).axes,...
                    'color',         FigOpt.sensor.colors.raw,...
                    'linestyle',     '-',...
                    'linewidth',     2, ...
                    'marker',        'none',...
                    'visible',       'off');
            end
            
            % observations
            for lmk = 1:size(Obs,2)
                SenFig(sen).drawn(lmk)   = false;
                SenFig(sen).measure(lmk) = line(...
                    'parent',    SenFig(sen).axes,...
                    'linestyle', '-',...
                    'linewidth', 3,...
                    'marker',    '.',...
                    'xdata',     [],...
                    'ydata',     [],...
                    'vis',       'off');
                SenFig(sen).mean(lmk) = line(...
                    'parent',    SenFig(sen).axes,...
                    'linestyle', '-',...
                    'marker',    'none',...
                    'xdata',     [],...
                    'ydata',     [],...
                    'vis',       'off');
                SenFig(sen).ellipse(lmk,1) = line(...
                    'parent',    SenFig(sen).axes,...
                    'xdata',     [],...
                    'ydata',     [],...
                    'vis',       'off');
                SenFig(sen).ellipse(lmk,2) = line(...
                    'parent',    SenFig(sen).axes,...
                    'xdata',     [],...
                    'ydata',     [],...
                    'vis',       'off');
                SenFig(sen).label(lmk) = text(...
                    'parent',             SenFig(sen).axes,...
                    'position',           [50 50],...
                    'string',             num2str(lmk),...
                    'color',              FigOpt.sensor.colors.label,...
                    'horizontalalignment','center',...
                    'vis',                'off');
            end
            
            % camera pinhole
            % --------------
        case {'omniCam'}
            % axes
            axis equal
            SenFig(sen).axes = gca;
            set(SenFig(sen).axes,...
                'position',      [0 0 1 1],...                  %[.07 .05 .9 .9],...
                'color',         FigOpt.sensor.colors.bckgnd,...
                'box',           'on',...
                'xlim',          [0 Sen(sen).par.imSize(1)],...
                'ylim',          [0,Sen(sen).par.imSize(2)],... % size of the image of sensor
                'alimmode',      'manual',...
                'climmode',      'manual',...
                'xaxislocation', 'top',...
                'ydir',          'reverse',...
                'layer',         'top',...
                'fontsize',      9,...
                'xcolor',        FigOpt.sensor.colors.axes,...
                'ycolor',        FigOpt.sensor.colors.axes,...
                'zcolor',        FigOpt.sensor.colors.axes);
            
            
            % raw points
            SenFig(sen).raw.points = line(...
                'parent',        SenFig(sen).axes,...
                'xdata',         [],...
                'ydata',         [],...
                'color',         FigOpt.sensor.colors.raw,...
                'linestyle',     'none',...
                'marker',        '+',...
                'markersize',    4);
            
            % observations
            for lmk = 1:size(Obs,2)
                SenFig(sen).drawn(lmk)   = false;
                SenFig(sen).measure(lmk) = line(...
                    'parent',    SenFig(sen).axes,...
                    'linestyle', '-',...
                    'linewidth', 3,...
                    'marker',    '.',...
                    'xdata',     [],...
                    'ydata',     [],...
                    'vis',       'off');
                SenFig(sen).mean(lmk) = line(...
                    'parent',    SenFig(sen).axes,...
                    'linestyle', '-',...
                    'marker',    'none',...
                    'xdata',     [],...
                    'ydata',     [],...
                    'vis',       'off');
                SenFig(sen).ellipse(lmk,1) = line(...
                    'parent',    SenFig(sen).axes,...
                    'xdata',     [],...
                    'ydata',     [],...
                    'vis',       'off');
                SenFig(sen).ellipse(lmk,2) = line(...
                    'parent',    SenFig(sen).axes,...
                    'xdata',     [],...
                    'ydata',     [],...
                    'vis',       'off');
                SenFig(sen).label(lmk) = text(...
                    'parent',             SenFig(sen).axes,...
                    'position',           [50 50],...
                    'string',             num2str(lmk),...
                    'color',              FigOpt.sensor.colors.label,...
                    'horizontalalignment','center',...
                    'vis',                'off');
            end
            
            
            % ADD HERE FOR INITIALIZATION OF NEW SENSORS's FIGURES
            % case {'newSensor'}
            % do something
            
            
            % unknown
            % -------
        otherwise
            % Print an error and exit
            error('??? Unknown sensor type ''%s''.',Sen(sen).type);
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

