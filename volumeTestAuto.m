function [result,baselineVal,newVal] = volumeTestAuto(imageFile1,varargin)
% VOLUMETEST is for the volume measurement accuracy quality control test.
% The function checks if the calculated volume is within 5% of the actual
% volume.

% Input parser
p = inputParser;
% Require at least one image 
addRequired(p,'imageFile1',@ischar);
% % Additional images
% for n = 2:10
%     addOptional(p,['imageFile' num2str(n)],@ischar);
% end
addOptional(p,'imageFile2',[],@ischar);
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
% Group image filenames in cell array


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



% Load images and read labels
for i = imageInputs
    imageFile = varargin{i};
    
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
    im = imread(imageFile);
%     figure;imshow(im);
    numImages = numel(imageInputs);
    if numImages <= 3
        gridSize = [1,numImages];
    elseif numImages <= 10
        gridSize = [2,round(numImages/2)];
    else
        gridSize = [3,round(numImages/3)];
    end
    subaxis(gridSize(1),gridSize(2),i,'Spacing',0.01,'Margin',0.01);
    imshow(im);
    hold on
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
set(fig,'Units','normalized','Position',[0.05 0.05 0.9 0.85]);

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