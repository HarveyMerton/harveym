function [isReflex] = IsReflex(vectStart,vectEnd,vectMid) 
% Finds if a curve is spans an angle greater than 180 degrees ie. if the
% curve is reflex. This is determined by finding if vectMid lies within the
% angle subtended by vectStart and vectEnd.

% Inputs: 
%   vectStart = position vector, relative to the centre of the curve, 
%       representing the start point of the curve.  
%   vectEnd = position vector, relative to the centre of the curve, 
%       representing the end point of the curve.  
%   vectMid = position vector, relative to the centre of the curve, 
%       representing another point on the curve.  

% Output: 
%   isReflex = 1 if curve spans an angle greater than 180 degrees, 0
%   if not, -1 if curve spans exactly 180 degrees. 

% Author: Harvey Merton 
% Date: 28/09/2018 


isReflex = 0;

% Find cross product of vectors representing start and end of
% the curve and the middle of the curve.
startX = cross(vectStart,vectMid);
endX = cross(vectEnd,vectMid);

% Find angle between vectors representing ends of the curve
% using: angle = arccos(a.b/|a||b|)

angleStartMid = acos(dot(vectStart,vectMid)/(norm(vectStart)*norm(vectMid)));
angleEndMid = acos(dot(vectEnd,vectMid)/(norm(vectEnd)*norm(vectMid)));


% If the sign of the z direction of startX and endX (xVectors) is the same, 
% cross product vectors point in the same direction.

% If xVectors point in the same direction, the middle of the
% curve cannot be in the small angle between vectors ie. must be reflex.
if sign(startX(3)) == sign(endX(3))
    isReflex = 1;
    
    % If the xVectors point in the opposite direction, vectMid is
    % either in the acute angle between vectStart and vectEnd, or
    % in the angle vertically opposite to this (ie. the curve must
    % be reflex).
    
    % vectMid will be in the small angle if angle between (vectStart
    % and vectMid) + angle between (vectEnd and vectMid) is <pi rad
elseif (angleStartMid + angleEndMid) > pi
    isReflex = 1;
elseif (angleStartMid + angleEndMid) == pi % If curve is an exact semi-circle
    isReflex = -1;
end

