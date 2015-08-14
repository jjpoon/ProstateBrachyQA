function [result,knownVal,measuredVal,freq] = areaTestAuto(imageFile,varargin)
% AREATEST is for the area measurement accuracy quality control test.
% The function checks if the calculated area is within 5% of the actual
% area value.

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

% Read frequency
freq = readFrequency(imageFile);

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
    if ~isempty(strfind(baselineFile{i,1},'Area'))
        knownVal = baselineFile{i,2};
    end
end

% Segment circle from image
[center,radius] = segmentCircle(imageFile);

% Code for plotting the segmented circle on original image
im_orig = imageFile;

if ~isempty(axesHandle)
    % Plot on specified axes if given as input
    parent = axesHandle;
else
    % Otherwise, plot on new figure
    figure;
    parent = gca;
end

% Plot image on axes
im = imshow(im_orig,'Parent',parent);
% Hold on
set(parent,'NextPlot','add');
c1 = viscircles(parent,center,radius);

if isempty(radius)
    % If no circle was found, set area to 0
    measuredVal = 0;
else
    % Convert radius to mm
    radius_mm = radius*pixelScale;
    % Convert radius to cm
    radius_cm = radius_mm/10;
    % Calculate area in cm
    area = pi*radius_cm^2;
    measuredVal = area;
end

% Figure title
% title(['Area = ' sprintf('%.2f',newVal) texlabel(' cm^2')]);
% Legend
l = [];
if ~isempty(c1)
    l = legend(c1,['Area: ' sprintf('%.2f',measuredVal) ' cm^2'],...
        'Location','southeast','Orientation','horizontal');
    % Change legend text and background colour
    set(l,'TextColor','w','Color',[0.2 0.2 0.2]);
end

% Callback when double clicking on image
set(im,'ButtonDownFcn',@(obj,eventdata)showInFigure(parent,l));

disp(['Known value: ' sprintf('%.2f',knownVal) ' cm^2']);
disp(['Measured value: ' sprintf('%.2f',measuredVal) ' cm^2']);

error = abs(measuredVal-knownVal);
% Compare measured area and known area
if isempty(error)
    result = [];
    disp('Missing information - could not complete test');
elseif error > 0.05*knownVal
    % Fail
    result = 0;
    disp('Area measurement accuracy test: failed');
else
    % Pass
    result = 1;
    disp('Area measurement accuracy test: passed');
end

end