% TRACKIMPORTER SCRIPT

% PURPOSE: 
% Allows conversion of F:SAE track data into data that the lap sim  
% program can use.


% OPERATION OVERVIEW: 
% Takes map data of a specific F:SAE race track and plots 
% the outline in a matlab graph where x and y are metres from a reference
% point at the bottom left corner of the track. 
% User must select points on the track to define curves and straights which
% are then used in calculations to find required data. 

 
% OPERATING INSTRUCTIONS: 
% 1. Decide on the specific section of track/track feature that you want to
% convert to lap sim track data.
% 2. Zoom to the desired section of the map and press any key.  

% For a STRAIGHT: 
% 3. Select the start point of the straight by pressing the any mouse or
% keyboard button (except ENTER) when the crosshairs are over the desired point. 
% 4. Select the end point of the straight using the same process. 

% For a CURVE: 
% 3. Select the start point of the curve by pressing any mouse 
% or keyboard button (except ENTER) when the crosshairs are over the desired point.
% 4. Select the end point of the curve using the same process. 
% 5. Select any other point on the curve between the start and end points
% by positioning the crosshairs over the desired point and pressing 'r'.  

% 5/6. Press enter and repeat for as many features on the track to import. 
% 6/7. When finished entering the last feature, close the track map and the
% program will terminate. 

% Author: Harvey Merton 
% Date: 24/09/2018


clearvars;

TRACK_WIDTH = 3; %Track width in metres

%Import track and plot
mapData = gpxread('Aus2016.gpx');

% Find the minimum latitude and longitude of the track and set this as the
% origin (for later conversion to "flat Earth" reference).
origin = [min(mapData.Latitude),min(mapData.Longitude)];

% Convert latitude and longitude map data to "flat Earth" data. 
% Data is transposed as lla2flat requires mx3 array.
flatMap = [mapData.Latitude; mapData.Longitude; zeros(1,length(mapData.Latitude))]';
flatMap = lla2flat(flatMap,origin,0,0);

% Plot track where x-axis and y-axis are metres from origin
trackHandle = figure(1);
plot(flatMap(:,2),flatMap(:,1));
title('Track Map');

% Counter of number of features selected on the map
numFeatures = 1;

while ishandle(trackHandle)
    pause; % Allow the user to zoom to the correct position on the map
    
    if ~ishandle(trackHandle) % Ensures track wasn't closed whilst waiting for user to zoom
        break;
    end
    
    % Allow the user to select points on the map to define boundaries of a 
    % feature. 
    try
        [x,y,button] = ginput(); % Gets point inputs from the user
    catch
        break; % Points will not be recorded if user exits map without pressing enter
    end
    
    % Store x&y point data in a single, 2D array
    clearvars points;
    points(:,1) = x; 
    points(:,2) = y;
    
    % Determine which feature was selected and ensure input was valid for  
    % this feature.
    try
        if size(points,1) == 2 % Feature selected must be a straight
            features{numFeatures} = NewStraight(points);
            numFeatures = numFeatures + 1;
            
        elseif size(points,1) == 3 % Feature selected must be a curve
            % Input validation - ensures curve was input in expected fashion 
            rNum = 0; % Number of points selected with 'r' - should be 1 middle point
            
            for i = 1:3 % Check how many points were entered with 'r'
                if button(i) == 'r' 
                    % Move r point to end of points array
                    temp = points(3,:); 
                    points(3,:) = points(i,:);
                    points(i,:) = temp;
                    
                    rNum = rNum +1;
                end 
            end 
            
            if rNum ~= 1 % If 0 or more than 1 point were entered with 'r', displays warning message
                fprintf(2,['Invalid input: must select 1 point on the curve with "r". ' ... 
                    'Start/stop points of the curve can be selected with any key.\n']); 
                continue; % Incorrect input is not used - ask for new input
            end
            
            features{numFeatures} = NewCorner(points,TRACK_WIDTH);
            numFeatures = numFeatures + 1;
            
        else % 1 or >=4 points were selected - invalid
           fprintf(2,['Invalid input: Select 2 points for a straight and 3 '...
           'points for a curve. \n']); 
        end
    
   
    catch % Will be executed if circfit has 3 co-linear points 
        fprintf(2,['Input Error. Please ensure points are selected and no '...
        'points are identical or, for a curve, co-linear \n']);
    end 
end

for i = 1:(numFeatures-1) 
    trackFileId = fopen('trackData.txt','a');
    fprintf(trackFileId, 'trackData{%d} = ',i);
    
    nextFeature = features{i};
    nextFeatureName = nextFeature{1};
    nextFeatureInfo = nextFeature{2};
    
    if strcmp(nextFeatureName,'Straight') 
        fprintf(trackFileId, '{\''Straight''; %f} \n',nextFeatureInfo);
    elseif strcmp(nextFeatureName,'Corner')
        fprintf(trackFileId, '{\''Corner''; [%f %f %f]} \n',nextFeatureInfo);
    end
end

