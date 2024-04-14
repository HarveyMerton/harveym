function [newCorner] = NewCorner(points, trackWidth)
% Creates a new track feature - curve. This is done by fitting the input
% points to a curve then calculating the radius of that curve and the angle
% between the end points. Uses this data and the track width to create a
% curve 'cell' in the format required by the LapSim program.

% Input:
%   points = 3x2 array containing x,y co-ordinates of 3 points which define
%       3 points on a curve. The first two points define the ends of the
%       curve while the third defines another point. First column = x
%       values, second column = y values.
% Output:
%   newStraight = 2x1 cell array of format:
%   {'Corner'; [minimumRadius angularDisplacement widthOfTrack]}

% Author: Harvey Merton
% Date: 28/09/2018


% Find centre and radius of curve. 
% circfit function retrieved from: 
% https://au.mathworks.com/matlabcentral/fileexchange/56412-circfit
[rad,xc,yc] = circfit(points(:,1),points(:,2)); % rad = radius, xc & yc = x and y values of circle centre
cCentre = [xc,yc];

% Convert track points to displacement vectors measured
% relative to the curve centre. Vectors must have 3 elements for cross(). 
vectStart = [(points(1,:) - cCentre),0];
vectEnd = [(points(2,:) - cCentre),0];
vectMid = [(points(3,:) - cCentre),0];

% Find if the curve extends over 180 degrees
isReflex = IsReflex(vectStart,vectEnd,vectMid);

% Find angle that curve turns through (angDisp)
angDisp = acos(dot(vectStart,vectEnd)/(norm(vectStart)*norm(vectEnd)));

if isReflex == -1 % If curve is an exact semi-circle
    angDisp = pi;
elseif isReflex ==1 % If angle is reflex, must be calculated by 2pi - angle between start and end vectors
    angDisp = 2*pi - angDisp;
end

newCorner = {'Corner';[rad,angDisp,trackWidth]};

end
