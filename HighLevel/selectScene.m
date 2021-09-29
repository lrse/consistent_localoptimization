
%switch between different defined scenes
switch scene
     case 'simpleSquare'
        position = [0;0;0];
        orientationDegrees = [0;0;0];
         
        mapSizeX = 10;
        
        mapMinX = -mapSizeX;
        mapMaxX = mapSizeX;
        mapMinY = -5;
        mapMaxY = 15;
        mapSizeZ = 5;
        World = struct(...
                'points',           thickCloister(-6,6,-1,11,1,7),... % 3d point landmarks - see THICKCLOISTER. 
                'segments',         []);  % 3D segments - see HOUSE. 
        dxSteps = repmat([0.08;0;0], [1 Time.lastFrame]);
        daSteps = repmat([0;0;0.9],  [1 Time.lastFrame]);
        
     case 'simpleReverseSquare'
        position = [0;0;0];
        orientationDegrees = [0;0;0];
         
        mapSizeX = 10;
        
        mapMinX = -mapSizeX;
        mapMaxX = mapSizeX;
        mapMinY = -5;
        mapMaxY = 15;
        mapSizeZ = 5;
        World = struct(...
                'points',           thickCloister(-6,6,-1,11,1,7),... % 3d point landmarks - see THICKCLOISTER. 
                'segments',         []);  % 3D segments - see HOUSE. 
        
        % loop forward
        dxSteps = repmat([0.08;0;0], [1 Time.lastFrame/2]);
        daSteps = repmat([0;0;0.9],  [1 Time.lastFrame/2]);
        
        % turn around
        dxSteps = [dxSteps [0;0;0]];
        daSteps = [daSteps [0;0;180]];
        
        % loop in reverse
        dxSteps = [dxSteps repmat([0.08;0;0], [1 Time.lastFrame/2])];
        daSteps = [daSteps repmat([0;0;-0.9],  [1 Time.lastFrame/2])];
        
     case 'quadraSquare'
        position = [0;0;0];
        orientationDegrees = [0;0;0];
        
        mapSizeX = 40;
        mapSizeY = 40;
        
        mapMinX = -mapSizeX;
        mapMaxX = mapSizeX;
        mapMinY = -mapSizeY;
        mapMaxY = mapSizeY;
        mapSizeZ = 5;
        squareSize = 35;
        
        World = struct(...
        'points', unique([thickCloister(-squareSize,0,-squareSize,0,1,7), thickCloister(-squareSize,0,0,squareSize,1,7),... 
                          thickCloister(0,squareSize,-squareSize,0,1,7), thickCloister(0,squareSize,0,squareSize,1,7)]','rows','stable')',... % 3d point landmarks - see THICKCLOISTER. 
        'segments',         []);  % 3D segments - see HOUSE. 
        % List of point that the robot has to go through
        a = squareSize - 10;
        controlPoints = [0,0,0; 0,-a,0;-a,-a,0;...
            -a,0,0;0,0,0;a,0,0;a,a,0;...
            0,a,0;0,0,0;0,-a,0;a,-a,0;...
            a,0,0; -a,0,0; -a,a,0;...
            0,a,0;0,0,0;]';
        % Use splines to stimate delta steps for each time
        [dxSteps, daSteps, firstZAngle] =  createRobotTrajectory(Time.lastFrame, controlPoints);
        
        % Defines starting z angle
        orientationDegrees(3) = firstZAngle;
end
