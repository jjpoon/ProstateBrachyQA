function [result,baselineVal,newVal] = volumeTestAuto(imageFile1,varargin)
% VOLUMETEST is for the volume measurement accuracy quality control test.
% The function checks if the calculated volume is within 5% of the actual
% volume.

% Input parser
p = inputParser;
% Require at least one image
addRequired(p,'imageFile1',@ischar);
% Variables for storing additional images
stringInputs = varargin(cellfun(@ischar,varargin));
% Get inputs ending with these image file extensions
fileExts = '^.+((\.bmp)|(\.jpg)|(\.tif)|(\.gif)|(\.dcm))$';
imageInputs = regexp(stringInputs,fileExts,'match');
imageInputs = [imageInputs{:}]';
% Add the first required image filename to list of images
imageInputs = [{imageFile1};imageInputs];
% Add enough optional inputs for the number of images given
for n = 2:numel(imageInputs)
    addOptional(p,['imageFile' num2str(n)],[],@ischar);
end
addParameter(p,'UpperScale',[]);
addParameter(p,'LowerScale',[]);
% Default step size 0.5 cm
addParameter(p,'StepSize',0.5);
addParameter(p,'AxesHandle',[]);
% Parse inputs
parse(p,imageFile1,varargin{:});
upper = p.Results.UpperScale;
lower = p.Results.LowerScale;
stepSize = p.Results.StepSize;
axesHandle = p.Results.AxesHandle;

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
    if strcmp(baselineVals{i,1},'Volume')
        baselineVal = baselineVals{i,2};
    end
end

if ~isempty(axesHandle)
    % Plot on specified axes if given as input
    parent = axesHandle;
else
    % Otherwise, plot on new figure
    fig = figure;
    parent = fig;
end

% Clear axes first
clf(parent);

% Load images and read labels
for i = 1:numel(imageInputs)
    imageFile = imageInputs{i};
    
    if isempty(upper) && isempty(lower)
        % Get the pixel to mm conversion ratio, automatically reading scale
        % labels from the image
        pixelScale = getPixelScale(imageFile);
    else
        % Get pixel scale using scale readings inputted by user
        pixelScale = getPixelScale(imageFile,upper,lower);
    end
    
    % Segment circle from image
    [center,radius] = segmentCircle(imageFile);
    
    % Code for plotting the segmented circle on original image
    im_orig = imread(imageFile);
    
%     if ~isempty(axesHandle)
%         % Plot on specified axes if given as input
%         parent = axesHandle;
%     else
%         % Otherwise, plot on new figure
%         parent = gca;
%     end
%     
%     % Clear axes first
%     cla(parent);
%     % Plot image on axes
%     imshow(im_orig,'Parent',parent);
%     % Hold on
%     set(parent,'NextPlot','add');
    
    numImages = numel(imageInputs);
    % Set size of image grid based on number of inputted images
    gridSize = [floor(sqrt(numImages)),ceil(sqrt(numImages))];
    n = gridSize(1);
    m = gridSize(2);
    [c,r] = ind2sub([gridSize(2) gridSize(1)], i);
    % Create subplots
    subplot('Position', [(c-1)/m, 1-(r)/n, 1/m, 1/n],'Parent',parent);
    imshow(im_orig);
    hold on
    % Visualize the segmented circle
    c1 = viscircles(center,radius);
    
    if isempty(radius)
        % If no circle was found, set area to 0
        areas(i) = 0;
    else
        % Convert radius to mm
        radius_mm = radius*pixelScale;
        % Convert radius to cm
        radius_cm = radius_mm/10;
        % Calculate area in cm
        areas(i) = pi*radius_cm^2;
    end
    
    % Legend
    l = legend(c1,['Area: ' sprintf('%.2f',areas(i)) ' cm^2'],...
        'Location','southeast','Orientation','horizontal');
    % Decrease legend marker size
    markerObjs = findobj(get(l,'children'), 'type', 'line');
    set(markerObjs, 'Markersize', 12);
    % Change legend text and background colour
    set(l,'TextColor','w','Color',[0.2 0.2 0.2]);
end

% Resize and position figure
if exist('fig','var')
    set(parent,'Units','normalized','Position',[0.05 0.05 0.9 0.85]);
end

% Calculate volume
vol = sum(areas)*stepSize;
newVal = vol;

disp(['Areas (cm^2): ' sprintf('%.2f  ',areas)]);
disp(['Step size: ' sprintf('%.2f mm',stepSize*10)]);
disp(['Baseline value: ' sprintf('%.2f',baselineVal) ' cm^3']);
disp(['New value: ' sprintf('%.2f',newVal) ' cm^3']);

error = abs(newVal-baselineVal);
% Compare measured volume and known volume
if error > 0.05*baselineVal
    % Fail
    result = 0;
    disp('Volume measurement accuracy test: failed');
else
    % Pass
    result = 1;
    disp('Volume measurement accuracy test: passed');
end

end