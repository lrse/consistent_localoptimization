function [dx,da,initZAngle] = createRobotTrajectory(totalFrames, xyz)

% initialize variables 
dx= zeros(3, totalFrames);
da= zeros(3, totalFrames);

% Estimate the spline using the control points 
funcSpline =cscvn(xyz);      

% Calculate the domain of the spline
domSpline = fix(funcSpline.breaks(end))+1;
% valuate the spline in the totalFrame time
nFrame = linspace(0,domSpline,totalFrames+1);
spline = fnval(funcSpline,nFrame);

% Initialize first pose z angle
vec = spline(:,2)-spline(:,1);
anglePrev = atan2d(vec(2), vec(1));
initZAngle = anglePrev;

for i=1:1:totalFrames
   vec = spline(:,i+1)-spline(:,i);
   % calculate the norm between two posees   
   dx(1,i) = norm(vec);
   
   % estimate the angle in z direction between two vectors
   % the camera can only rotate in the z axes. 
   angle = atan2d(vec(2), vec(1));
   da(3,i) =  wrapTo180( angle - anglePrev);
   anglePrev = angle;
end

end

%%
% 
% spline = [0 0 0;...
%           0,1,0;...
%           0,2,0;...
%           1,3,0;...
%           2,3,0;...
%           3,4,0;...
%           3,5,0;...
%           2,5,0;...         
%           1,4,0]';
% 
% dx= zeros(3, length(spline)-1);
% da= zeros(3, length(spline)-1);
% angle = zeros(1, length(spline)+1);
% for i=1:1:length(spline)-1
%    dx(1,i) = norm(spline(:,i+1)-spline(:,i));
%    vec = spline(:,i+1)-spline(:,i);
%    angle(i+1) = atan2d(vec(2), vec(1))
%    da(3,i) = wrapTo180( angle(i+1) - angle(i));
% end
% dx
% da
