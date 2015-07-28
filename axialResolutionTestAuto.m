function [result,baselineVals,newVals] = axialResolutionTestAuto(imageFile,varargin)
% AXIALRESOLUTIONTEST is for the axial resolution quality control test.
% The function checks if the axial resolution has changed by
% more than 1 mm from the baseline value.

% Input parser
p = inputParser;
addRequired(p,'imageFile',@isnumeric);
addParameter(p,'UpperScale',[]);
addParameter(p,'LowerScale',[]);
addParameter(p,'AxesHandle',[]);
% Parse inputs
parse(p,imageFile,varargin{:});
upper = p.Results.UpperScale;
lower = p.Results.LowerScale;
axesHandle = p.Results.AxesHandle;

% If user has given the scale readings
if ~isempty(upper) && ~isempty(lower)
    % Get pixel scale using scale readings inputted by user
    pixelScale = getPixelScale(imageFile,upper,lower);
else
    % Get the pixel to mm conversion ratio, automatically reading scale
    % labels from the image
    pixelScale = getPixelScale(imageFile);
end

% Get baseline values
if ~exist('Baseline.mat','file')
    % Read xls file if mat file not created yet
    baselineFile = readBaselineFile('Baseline.xls');
else
    % Get baseline value from mat file (faster)
    load('Baseline.mat');
end

% Get baseline values for this test
for i = 1:size(baselineFile,1)
    if ~isempty(strfind(baselineFile{i,1},'Axial resolution'))
        baselineVals = [baselineFile{i,2:7}];
    end
end

% Crop to ultrasound image
im_orig = imageFile;
width = size(im_orig,2);
height = size(im_orig,1);
% Crop dimensions
cropX = round(0.15*width);
cropY = round(0.2*height);
cropWidth = round(0.7*width);
cropHeight = round(0.65*height);
im_cropped = im_orig(cropY:cropY+cropHeight,cropX:cropX+cropWidth);
% Make all non-black objects white
im_bw = im2bw(im_cropped,0.01);
% Get indices of rows/columns with nonzero elements
[row col] = find(im_bw);
% Crop tight to objects
im_tight = im_cropped(min(row):max(row),min(col):max(col));

% Store x and y offset for positioning cropped image relative to original image. 
xOffset = cropX + min(col) - 2;
yOffset = cropY + min(row) - 2;

% Filter image using wiener2 (2-D adaptive noise-removal filtering)
filt1 = wiener2(im_tight,[10 10]);
% Filter again using 2D �Prewitt� filter, emphasizing horizontal edges
filt2 = imfilter(filt1,fspecial('prewitt'));
% Convert to black and white, 0.2 threshold best for segmenting filaments
bw = im2bw(filt2,0.1);
% Remove unwanted white areas from edges of image (scale tick markings, top
% and bottom of ultrasound image)
bw(:,1:30) = 0;         % Remove left edge
bw(:,end-30:end) = 0;   % Remove right edge
bw(1:80,:) = 0;         % Remove top edge
bw(end-120:end,:) = 0;   % Remove bottom edge

% Remove large objects
bw = bw - bwareaopen(bw,150);
% Remove small objects
bw = bwareaopen(bw,20);

% Get the bw image region properties
bwRegions = regionprops(bw,'Centroid','MajorAxisLength','MinorAxisLength','Area','Orientation');

% Only keep regions with expected major axis length
lengths = [bwRegions.MajorAxisLength]';
filaments = find(lengths>8 & lengths<50);
bw = ismember(bwlabel(bw),filaments);
% Get the filament region properties
regions = regionprops(bw,'Centroid','MajorAxisLength','MinorAxisLength','Area','Orientation');

% Get centroids of filament regions
for n = 1:numel(regions)
    centroids(n,:) = regions(n).Centroid;
end

yCoords = centroids(:,2);
lengths = [regions.MajorAxisLength]';

% Assume image was taken in axial view at first
view = 'axial';

% Find indices of regions with similar y coordinate and similar area
for i = 1:numel(regions)
    % Get indices of regions with similar y coord
    similarY = find(yCoords>yCoords(i)-6 & yCoords<yCoords(i)+6);
    % Get indices of regions with similar length
    similarLength = find(lengths>lengths(i)-4 & lengths<lengths(i)+4);
    % Get indices of regions with similar y and length
    rowFilaments = intersect(similarY,similarLength);
    % Check left-to-right y difference for row filaments (look for outliers)
    if numel(rowFilaments) >= 6
        while any(abs(diff(yCoords(rowFilaments)))>3)
            diffs = abs(diff(yCoords(rowFilaments)))>3;
            % Find first outlier index
            ind1 = find(diffs,1);
            if ind1 == numel(diffs)
                outlier = ind1 + 1;
            else
                if diffs(ind1 + 1) == 1
                    outlier = ind1 + 1;
                else
                    outlier = ind1;
                end
            end
            % Remove outlier
            rowFilaments(outlier) = [];
            % If number of row filaments drops below 6, break
            if numel(rowFilaments) < 6
                break
            end
        end
    end
    % Check if found 6 similar regions
    if numel(rowFilaments) == 6
        % If there are too many regions with similar y, assume false positive
        if numel(similarY) < 10
            % Found filaments in row, image was taken in sagittal view
            view = 'sagittal';
            % Restrict regions to row filaments
            regions = regions(rowFilaments);
            % Restrict centroids to row filaments
            centroids = centroids(rowFilaments,:);
            break
        end
    end
end

% -------------------------------------------------------------------------
% Get filament indices depending on view
if strcmp(view,'axial')
    
    % Get the indices of the corner filaments
    
    % Top left
    topLeftCorner = [0 0];
    % Get distances from corner to centroids
    vectors = centroids - repmat(topLeftCorner,size(centroids,1),1);
    distances = sqrt(vectors(:,1).^2 + vectors(:,2).^2);
    [topLeft,topLeftInd] = min(distances);
    
    % Top right
    topRightCorner = [size(bw,2) 0];
    % Get distances from corner to centroids
    vectors = centroids - repmat(topRightCorner,size(centroids,1),1);
    distances = sqrt(vectors(:,1).^2 + vectors(:,2).^2);
    [topRight,topRightInd] = min(distances);
    
    % Bottom left
    bottomLeftCorner = [0 size(bw,1)];
    % Get distances from corner to centroids
    vectors = centroids - repmat(bottomLeftCorner,size(centroids,1),1);
    distances = sqrt(vectors(:,1).^2 + vectors(:,2).^2);
    [bottomLeft,bottomLeftInd] = min(distances);
    
    % Top right
    bottomRightCorner = [size(bw,2) size(bw,1)];
    % Get distances from corner to centroids
    vectors = centroids - repmat(bottomRightCorner,size(centroids,1),1);
    distances = sqrt(vectors(:,1).^2 + vectors(:,2).^2);
    [bottomRight,bottomRightInd] = min(distances);
    
    % Check orientation by using position of 6th filament (left of center)
    dist_topLeft = norm(regions(topLeftInd).Centroid - regions(6).Centroid);
    dist_bottomLeft = norm(regions(bottomLeftInd).Centroid - regions(6).Centroid);
    if dist_bottomLeft < dist_topLeft
        % bottomleft = B1, bottomright = F1, topleft = B5, topright = F5
        filamentIndices = [bottomLeftInd,bottomRightInd,topLeftInd,topRightInd];
    else
        % bottomleft = F1, bottomright = B1, topleft = F5, topright = B5
        filamentIndices = [bottomRightInd,bottomLeftInd,topRightInd,topLeftInd];
    end
    
elseif strcmp(view,'sagittal')
    
    % Get leftmost filament
    [left,leftInd] = min(centroids(:,1));
    % Get rightmost filament
    [right,rightInd] = max(centroids(:,1));
    
    % Check orientation by comparing distance between left 2 filaments and
    % distance between right 2 filaments
    dist_left2 = norm(regions(1).Centroid - regions(2).Centroid);
    dist_right2 = norm(regions(end-1).Centroid - regions(end).Centroid);
    if dist_left2 < dist_right2
        % left = 1, right = 6
        filamentIndices = [leftInd,rightInd];
    else
        % left = 6, right = 1
        filamentIndices = [rightInd,leftInd];
    end
    
end
% -------------------------------------------------------------------------

% Loop through filaments of interest
points = [];
minorAxisLengths = [];
for f = filamentIndices
    % Centroid of filament
    centroid = regions(f).Centroid;
    % Angle between x-axis and minor axis of region (neg. because MATLAB y
    % increases going down)
    theta = -(regions(f).Orientation+90);
    % Length of minor axis
    minorAxisLength = regions(f).MinorAxisLength;
    % Rotation matrix to get vector pointing along minor axis
    R = [cosd(theta) -sind(theta); sind(theta) cosd(theta)];
    % Vector pointing along minor axis
    v = R*[1;0];
    
    % Points for showing axial resolution
    points = [points; centroid + (minorAxisLength/2)*v'];
    points = [points; centroid - (minorAxisLength/2)*v'];
    
    % Store minor axis length
    minorAxisLengths = [minorAxisLengths minorAxisLength];
end

% -------------------------------------------------------------------------
% Plot markers on figure to show points that were found
% Subtract 2 from y offset because 'Prewitt' filter used above lowers image
% by 2 pixels
yOffset = yOffset - 2;
% Create column vectors for offsets, used for plotting markers
yOffset = repmat(yOffset,2,1);
xOffset = repmat(xOffset,2,1);

if ~isempty(axesHandle)
    % Plot on specified axes if given as input
    parent = axesHandle;
else
    % Otherwise, plot on new figure
    figure;
    parent = gca;
end

% Clear axes first
cla(parent);
% Plot image on axes
im = imshow(im_orig,'Parent',parent);
% Hold on
set(parent,'NextPlot','add');
% Plot markers
m1 = plot(xOffset+points(1:2,1),yOffset+points(1:2,2),...
    '+','MarkerSize',5,'Linewidth',2,'Color','r','Parent',parent);
m2 = plot(xOffset+points(3:4,1),yOffset+points(3:4,2),...
    '+','MarkerSize',5,'Linewidth',2,'Color','g','Parent',parent);
if strcmp(view,'axial')
    m3 = plot(xOffset+points(5:6,1),yOffset+points(5:6,2),...
        '+','MarkerSize',5,'Linewidth',2,'Color','c','Parent',parent);
    m4 = plot(xOffset+points(7:8,1),yOffset+points(7:8,2),...
        '+','MarkerSize',5,'Linewidth',2,'Color','y','Parent',parent);
end
% -------------------------------------------------------------------------

% Get the axial resolution at filaments of interest
newVals = minorAxisLengths.*pixelScale;

% Figure title
% title(['Axial resolution = ' sprintf('%.2f',newVal) ' mm']);
% Legend
if strcmp(view,'axial')
    l = legend([m1,m2,m3,m4],['Dist: ' sprintf('%.2f',minorAxisLengths(1)*pixelScale) ' mm'],...
                         ['Dist: ' sprintf('%.2f',minorAxisLengths(2)*pixelScale) ' mm'],...
                         ['Dist: ' sprintf('%.2f',minorAxisLengths(3)*pixelScale) ' mm'],...
                         ['Dist: ' sprintf('%.2f',minorAxisLengths(4)*pixelScale) ' mm'],...
                         'Location','southeast','Orientation','horizontal');
elseif strcmp(view,'sagittal')
    l = legend([m1,m2],['Dist: ' sprintf('%.2f',minorAxisLengths(1)*pixelScale) ' mm'],...
                         ['Dist: ' sprintf('%.2f',minorAxisLengths(2)*pixelScale) ' mm'],...
                         'Location','southeast','Orientation','horizontal');
end

% Decrease legend marker size
markerObjs = findobj(get(l,'children'), 'type', 'line');
set(markerObjs, 'Markersize', 12);
% Change legend text and background colour
set(l,'TextColor','w','Color',[0.2 0.2 0.2]);

% Callback when double clicking on image
set(im,'ButtonDownFcn',@(obj,eventdata)showInFigure(parent,l));

if strcmp(view,'axial')
    % Display measurements for axial view
    disp('Baseline values (axial plane):');
    disp(['Proximal (left): ' sprintf('%.2f',baselineVals(1)) ' mm']);
    disp(['Proximal (right): ' sprintf('%.2f',baselineVals(2)) ' mm']);
    disp(['Distal (left): ' sprintf('%.2f',baselineVals(3)) ' mm']);
    disp(['Distal (right): ' sprintf('%.2f',baselineVals(4)) ' mm']);
    disp(' ');
    disp('New values (axial plane):');
    disp(['Proximal (left): ' sprintf('%.2f',newVals(1)) ' mm']);
    disp(['Proximal (right): ' sprintf('%.2f',newVals(2)) ' mm']);
    disp(['Distal (left): ' sprintf('%.2f',newVals(3)) ' mm']);
    disp(['Distal (right): ' sprintf('%.2f',newVals(4)) ' mm']);
    disp(' ');
else
    % Display measurements for sagittal view
    disp('Baseline values (longitudinal plane):');
    disp(['Proximal: ' sprintf('%.2f',baselineVals(5)) ' mm']);
    disp(['Distal: ' sprintf('%.2f',baselineVals(6)) ' mm']);
    disp(' ');
    disp('New values (longitudinal plane):');
    disp(['Proximal: ' sprintf('%.2f',newVals(1)) ' mm']);
    disp(['Distal: ' sprintf('%.2f',newVals(2)) ' mm']);
    disp(' ');
end

% Change in axial resolution (in mm)
if strcmp(view,'axial')
    change = abs(newVals - baselineVals(1:4));
else
    change = abs(newVals - baselineVals(5:6));
end
% Get results (0 or 1 if fail or pass requirement)
result = change<=1;
% Check if axial resolution has changed by more than 1 mm
if any(change > 1)
    % Fail
    disp('Axial resolution test: failed');
else
    % Pass
    disp('Axial resolution test: passed');
end

end