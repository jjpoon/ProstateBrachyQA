function [result,baselineVal,newVal] = depthTestAuto(imageFile,varargin)
% DEPTHTEST is for the depth of penetration quality control test.
% The function checks if the maximum depth of pentration has changed by
% more than 1 cm from the baseline value.

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
    baselineVals = readBaselineFile('Baseline.xls');
else
    % Get baseline value from mat file (faster)
    load('Baseline.mat');
end

% Get baseline value for this test
for i = 1:size(baselineVals,1)
    if strcmp(baselineVals{i,1},'Depth of penetration')
        baselineVal = baselineVals{i,2};
    end
end

% Measure depth penetration
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

% Convert to black and white
im = im2bw(im_tight,0.2);
% Get region in image
regions = regionprops(im);
% Get areas and find the index of the largest region
areas = [regions.Area];
[maxArea,ind] = max(areas);
% Get boundingBox showing depth of penetration
boundingBox = regions(ind).BoundingBox;

% Depth of penetration is equal to the height of the bounding box
% Depth in pixels
depth_pixels = boundingBox(4);
% Depth in mm
depth_mm = depth_pixels*pixelScale;
newVal = depth_mm;

% Plot markers to show points that were found
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
% Plot bounding box
r1 = rectangle('Position',[xOffset+boundingBox(1),yOffset+boundingBox(2),boundingBox(3:4)],...
    'Linewidth', 1, 'EdgeColor', 'r', 'LineStyle', '--','Parent',parent);
% Plot marker
m1 = plot(xOffset+boundingBox(1)+boundingBox(3)/2,yOffset+boundingBox(2),...
    '+','MarkerSize',15,'Linewidth',2,'Color','c','Parent',parent);

% Figure title
% title(['Depth = ' sprintf('%.2f',newVal) ' mm']);
% Legend
l = legend(m1,['Depth: ' sprintf('%.2f',newVal) ' mm'],'Location','southeast');
% Decrease legend marker size
markerObjs = findobj(get(l,'children'), 'type', 'line');
set(markerObjs, 'Markersize', 12);
% Change legend text and background colour
set(l,'TextColor','w','Color',[0.2 0.2 0.2]);

disp(['Baseline value: ' sprintf('%.2f',baselineVal) ' mm']);
disp(['New value: ' sprintf('%.2f',newVal) ' mm']);

% Change in max depth (in cm)
change = abs(newVal - baselineVal)/10;
% Check if max depth has changed by more than 1 cm
if change > 1
    % Fail
    result = 0;
    disp('Depth of penetration test: failed');
else
    % Pass
    result = 1;
    disp('Depth of penetration test: passed');
end

end