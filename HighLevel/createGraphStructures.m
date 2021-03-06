function [Rob,Sen,Raw,Lmk,Obs,Trj,Frm,Fac,Tim] = createGraphStructures(Robot,Sensor,Time,Opt)

% CREATEGRAPHSTRUCTURES  Initialize graphSLAM data structures from user data.
%
%   See also CREATESLAMSTRUCTURES.

%   Copyright 2015 Joan Sola @ IRI-UPC-CSIC.

global Map

% Clear ID factory
clear newId; 

% HARDWARE
% Create robots and controls
Rob = createRobots(Robot);

% Create sensors
[Sen,Raw] = createSensors(Sensor);

% Install sensors in robots
[Rob,Sen] = installSensors(Rob,Sen);

% Create Landmarks and non-observables
Lmk = createLandmarks(Opt);

% ESTIMATION
% Create Map - empty
Map = createMap(Rob,Sen,Opt);

% Create robot trajectories
Trj = createTrajectory(Rob, Opt);

% Create frames
Frm = createFrames(Rob,Trj);

% Create factors, Compute max nbr of factors based on general options
numFactors = Opt.map.numFrames*(2 + Opt.correct.nUpdates + Opt.init.nbrInits(2)) + Opt.init.nbrInits(1);
Fac = createFactors(numFactors);

% Create Observations (matrix: [ line=sensor , colums=landmark ])
Obs = createObservations(Sen,Opt);

% Initialize robots in Map
Rob = initRobots(Rob);

% Create time variables
Tim = createTime(Time);

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

