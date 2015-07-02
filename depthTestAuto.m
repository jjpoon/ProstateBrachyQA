function [result,baselineVal,newVals] = depthTestAuto(imageFile,varargin)
% DEPTHTEST is for the depth of penetration quality control test.
% The function checks if the maximum depth of pentration has changed by
% more than 1 cm from the baseline value.

% Input parser
p = inputParser;
addRequired(p,'imageFile',@ischar);
addParameter(p,'UpperScale',[]);
addParameter(p,'LowerScale',[]);
addParameter(p,'AxesHandle',[]);
addParameter(p,'Plane',@ischar);
% Parse inputs
parse(p,imageFile,varargin{:});
upper = p.Results.UpperScale;
lower = p.Results.LowerScale;
axesHandle = p.Results.AxesHandle;
plane = p.Results.Plane;

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
    if ~isempty(strfind(baselineFile{i,1},'Depth of penetration'))
        baselineVals = [baselineFile{i,2:3}];
    end
end
% Check what plane (axial or longitudinal) image was taken in and choose
% the corresponding baseline value
if strcmpi(plane,'longitudinal')
    baselineVal = baselineVals(2);
else
    baselineVal = baselineVals(1);
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

% Horizontal edge-emphasizing filter
filt = imfilter(im_tight,fspecial('Prewitt'));
% 2D median filtering
med = medfilt2(filt);
% Convert to black and white
bw = im2bw(med,0.1);
% Remove large regions
bw = bw - bwareaopen(bw,30);
% Dilate image to make large region
bw = imdilate(bw,strel('disk',10));
bw = im2bw(bw);
% Get largest region in image
regions = regionprops(bw);
% Get areas and find the index of the largest region
areas = [regions.Area];
[maxArea,ind] = max(areas);
% Get boundingBox showing depth of penetration
boundingBox = regions(ind).BoundingBox;
% Set coordinates of top point for showing depth of penetration
topPoint = [round(size(bw,2)/2),boundingBox(2)];

% Get bottom point to measure depth distance from (top of semi-circle)
im_bw = im2bw(im_tight,0.01);
im_bw = bwareaopen(im_bw,10000);
y = find(im_bw(:,round(size(im_bw,2)/2)),1,'last');
bottomPoint = [size(im_bw,2)/2,y];

% Depth in pixels
depth_pixels = bottomPoint(2) - topPoint(2);
% Depth in mm
depth_mm = depth_pixels*pixelScale;
newVals = depth_mm;

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
im = imshow(im_orig,'Parent',parent);
% Hold on
set(parent,'NextPlot','add');
% Plot bounding box
% r1 = rectangle('Position',[xOffset+boundingBox(1),yOffset+boundingBox(2),boundingBox(3:4)],...
%     'Linewidth', 1, 'EdgeColor', 'r', 'LineStyle', '--','Parent',parent);
% Plot marker
m1 = plot(xOffset+boundingBox(1)+boundingBox(3)/2,yOffset+boundingBox(2),...
    '+','MarkerSize',15,'Linewidth',2,'Color','c','Parent',parent);
% m1 = plot(xOffset+topPoint(1),yOffset+topPoint(2),...
%     '+','MarkerSize',15,'Linewidth',2,'Color','c','Parent',parent);
% m2 = plot(xOffset+bottomPoint(1),yOffset+bottomPoint(2),...
%     '+','MarkerSize',15,'Linewidth',2,'Color','r','Parent',parent);
% Figure title
% title(['Depth = ' sprintf('%.2f',newVal) ' mm']);
% Legend
l = legend(m1,['Depth: ' sprintf('%.2f',newVals) ' mm'],'Location','southeast');
% Decrease legend marker size
markerObjs = findobj(get(l,'children'), 'type', 'line');
set(markerObjs, 'Markersize', 12);
% Change legend text and background colour
set(l,'TextColor','w','Color',[0.2 0.2 0.2]);

% Callback when double clicking on image
set(im,'ButtonDownFcn',@(obj,eventdata)showInFigure(parent,l));

disp(['Baseline value: ' sprintf('%.2f',baselineVal) ' mm']);
disp(['New value: ' sprintf('%.2f',newVals) ' mm']);

% Change in max depth (in cm)
change = abs(newVals - baselineVal)/10;
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