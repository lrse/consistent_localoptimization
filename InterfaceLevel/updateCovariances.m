function [Rob,Lmk,Frm] = updateCovariances(Rob,Lmk,Frm)

% UPDATECOVARIANCES Update Frm and Lmk covariances.
%   [Rob,Lmk,Frm] = UPDATECOVARIANCES(Rob,Lmk,Frm) updates the covariances
%   of robots Rob, landmarks Lmk and frames Frm, by taking resulting
%   diagonal of the problem covariance Map.P(Xxx.state.rq, Xxx.state.rq)
%   and propagate it through the inverse manifold convertion function.
%
%   Each state is updated following a different procedure depending on its
%   type.
%
%   See also UPDATEKEYFRM, UPDATESTATES.

% Copyright 2015-     Joan Sola @ IRI-UPC-CSIC.


global Map

for rob = [Rob.rob]
    for frm = [Frm(rob,[Frm(rob,:).used]).frm]
        if ~all(Map.used(Frm(rob,frm).state.r)) || (isfield(Map,'toSchur') && ~isempty(Map.toSchur) && all(ismember(Frm(rob,frm).state.r,Map.toSchur)))
            continue
        end
        
        Frm(rob,frm).state.P = full(Frm(rob,frm).state.M * Map.P(Frm(rob,frm).state.r,Frm(rob,frm).state.r) * Frm(rob,frm).state.M');
    end
end
for lmk = [Lmk([Lmk.used]).lmk]
    if ~all(Map.used(Lmk(lmk).state.r)) || (isfield(Map,'toSchur') && ~isempty(Map.toSchur) && all(ismember(Lmk(lmk).state.r, Map.toSchur)))
        continue
    end
    
    switch Lmk(lmk).type
        case 'eucPnt'
            % Trivial composition -- no manifold stuff
            Lmk(lmk).state.P = full(Map.P(Lmk(lmk).state.r,Lmk(lmk).state.r));
        otherwise
            error('??? Unknown landmark type ''%s'' or Update not implemented.',Lmk.type)
    end
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

