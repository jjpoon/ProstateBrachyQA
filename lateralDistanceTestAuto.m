function [result,knownVal,measuredVals] = lateralDistanceTestAuto(imageFile,varargin)
% LATERALDISTANCETEST is for the lateral distance measurement accuracy quality control test.
% The function compares the lateral distance measurement with the known
% value and checks if the error is larger than 3 mm (absolute) or 3% (relative).

% Input parser
p = inputParser;
addRequired(p,'imageFile',@ischar);
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

% Get baseline value for this test
for i = 1:size(baselineFile,1)
    if ~isempty(strfind(baselineFile{i,1},'Lateral distance'))
        knownVal = baselineFile{i,2};
    end
end

% Crop to ultrasound image
im_orig = imread(imageFile);
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
% Filter again using 2D ‘Prewitt’ filter, emphasizing horizontal edges
filt2 = imfilter(filt1,fspecial('prewitt'));
% Convert to black and white, 0.2 threshold best for segmenting filaments
bw = im2bw(filt2,0.2);
% Remove unwanted white areas from edges of image (scale tick markings, top
% and bottom of ultrasound image)
bw(:,1:30) = 0;         % Remove left edge
bw(:,end-30:end) = 0;   % Remove right edge
bw(1:40,:) = 0;         % Remove top edge
bw(end-120:end,:) = 0;   % Remove bottom edge

% Remove large objects
bw = bw - bwareaopen(bw,100);
% Remove small objects
bw = bwareaopen(bw,5);

% Get the filament region properties
regions = regionprops(bw,'Centroid','MinorAxisLength','Area','Orientation');

% Get centroids of filament regions
for n = 1:numel(regions)
    centroids(n,:) = regions(n).Centroid;
end

yCoords = centroids(:,2);
areas = [regions.Area]';

% Assume image was taken in axial view at first
view = 'axial';

% Find indices of regions with similar y coordinate and similar area
for i = 1:numel(regions)
    % y coordinate of current region
    yCoord = yCoords(i);
    % area of current region
    area = areas(i);
    % Get indices of regions with similar y coord
    similarY = find(yCoords>yCoord-10 & yCoords<yCoord+10);
    % Get indices of regions with similar area
    similarArea = find(areas>area-20 & areas<area+20);
    % Get indices of regions with both similar y coord and area
    rowFilaments = intersect(similarY,similarArea);
    % Check if number of similar regions found is between 5 and 6
    if numel(rowFilaments) >= 5 && numel(rowFilaments) <= 6
        % Found filaments in row, image was taken in sagittal view
        view = 'sagittal';
        % Restrict regions to row filaments
        regions = regions(rowFilaments);
        % Restrict centroids to row filaments
        centroids = centroids(rowFilaments,:);
        break
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
    [topLeftDist,topLeftInd] = min(distances);
    
    % Top right
    topRightCorner = [size(bw,2) 0];
    % Get distances from corner to centroids
    vectors = centroids - repmat(topRightCorner,size(centroids,1),1);
    distances = sqrt(vectors(:,1).^2 + vectors(:,2).^2);
    [topRightDist,topRightInd] = min(distances);
    
    % Bottom left
    bottomLeftCorner = [0 size(bw,1)];
    % Get distances from corner to centroids
    vectors = centroids - repmat(bottomLeftCorner,size(centroids,1),1);
    distances = sqrt(vectors(:,1).^2 + vectors(:,2).^2);
    [bottomLeftDist,bottomLeftInd] = min(distances);
    
    % Top right
    bottomRightCorner = [size(bw,2) size(bw,1)];
    % Get distances from corner to centroids
    vectors = centroids - repmat(bottomRightCorner,size(centroids,1),1);
    distances = sqrt(vectors(:,1).^2 + vectors(:,2).^2);
    [bottomRightDist,bottomRightInd] = min(distances);
    
    filamentIndices = [bottomLeftInd,bottomRightInd,topLeftInd,topRightInd];
    
elseif strcmp(view,'sagittal')
    
    % Get leftmost filament
    [left,leftInd] = min(centroids(:,1));
    % Get rightmost filament
    [right,rightInd] = max(centroids(:,1));
    
    filamentIndices = [leftInd,rightInd];
    
end
% -------------------------------------------------------------------------

% Loop through filaments of interest
points = [];
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
    
    % Point on top of filament for showing lateral distance
    points = [points; centroid + (minorAxisLength/2)*v'];
end

% -------------------------------------------------------------------------
% Plot markers on figure to show points that were found
% Subtract 2 from y offset because 'Prewitt' filter used above lowers image
% by 2 pixels
yOffset = yOffset - 2;
% Create column vectors for offsets, used for plotting markers
yOffset = repmat(yOffset,size(points,1),1);
xOffset = repmat(xOffset,size(points,1),1);

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
imshow(im_orig,'Parent',parent);
% Hold on
set(parent,'NextPlot','add');

% Get marker points corrected with x and y offset
markers = [xOffset+points(:,1),yOffset+points(:,2)];
% Plot lateral distance lines and markers
line1 = line(markers(1:2,1),markers(1:2,2),'LineStyle',':','Color','w','Parent',parent);
m1 = plot(markers(1:2,1),markers(1:2,2),...
    '+','MarkerSize',10,'Linewidth',2,'Color','r','Parent',parent);
if strcmp(view,'axial')
    line2 = line(markers(3:4,1),markers(3:4,2),'LineStyle',':','Color','w','Parent',parent);
    m2 = plot(markers(3:4,1),markers(3:4,2),...
        '+','MarkerSize',10,'Linewidth',2,'Color','g','Parent',parent);
end
% -------------------------------------------------------------------------

if strcmp(view,'axial')
    % Top lateral distance
    lateralDist1_pixels = norm(markers(2,:)-markers(1,:));
    lateralDist1_mm = lateralDist1_pixels*pixelScale;
    % Bottom lateral distance
    lateralDist2_pixels = norm(markers(4,:)-markers(3,:));
    lateralDist2_mm = lateralDist2_pixels*pixelScale;
    % Average lateral distance
    measuredVals = [lateralDist1_mm lateralDist2_mm];
elseif strcmp(view,'sagittal')
    % Lateral distance
    lateralDist_pixels = norm(markers(2,:)-markers(1,:));
    lateralDist_mm = lateralDist_pixels*pixelScale;
    measuredVals = lateralDist_mm;
end

% Figure title
% title(['Lateral distance = ' sprintf('%.2f',newVal) ' mm']);
% Legend
if strcmp(view,'axial')
    l = legend([m1,m2],['Dist: ' sprintf('%.2f',lateralDist1_mm) ' mm'],...
        ['Dist: ' sprintf('%.2f',lateralDist2_mm) ' mm'],...
        'Location','southeast','Orientation','horizontal');
elseif strcmp(view,'sagittal')
    l = legend(m1,['Dist: ' sprintf('%.2f',lateralDist_mm) ' mm'],...
        'Location','southeast','Orientation','horizontal');
end
% Decrease legend marker size
markerObjs = findobj(get(l,'children'), 'type', 'line');
set(markerObjs, 'Markersize', 12);
% Change legend text and background colour
set(l,'TextColor','w','Color',[0.2 0.2 0.2]);

disp(['Known value: ' sprintf('%.2f',knownVal) ' mm']);

if numel(measuredVals) > 1
    disp(' ');
    disp('Measured values:');
    disp(['Proximal: ' sprintf('%.2f',measuredVals(1)) ' mm']);
    disp(['Distal: ' sprintf('%.2f',measuredVals(2)) ' mm']);
    disp(' ');
else
    disp(['Measured value: ' sprintf('%.2f',measuredVals)]);
end

if ~isempty(knownVal)
    error = abs(measuredVals-repmat(knownVal,size(measuredVals)));
else
    error = [];
end
% Check measured lateral distance measurement errors
result = error<=3 | error<=0.03*knownVal;
if isempty(result)
    disp('Missing information - could not complete test');
elseif any(result == 0)
    % Fail
    disp('Lateral distance measurement accuracy test: failed');
else
    % Pass
    disp('Lateral distance measurement accuracy test: passed');
end

end