function [newStraight] = NewStraight(points) 
% Creates a new track feature - straight. This is done by calculating the
% distance of the straight and creating a straight 'cell' in the format 
% required by the LapSim program.

% Input: 
%   points = 2x2 array containing x,y co-ordinates of 2 points which define
%       either end of a straight. First column = x values, second column = y
%       values. 
% Output: 
%   newStraight = 2x1 cell array of format: 
%   {'Straight'; distance}

% Author: Harvey Merton
% Date: 28/09/2018


dist = sqrt((points(2,1)-points(1,1))^2 + (points(2,2)-points(1,2))^2);
newStraight = {'Straight';dist};

end
            