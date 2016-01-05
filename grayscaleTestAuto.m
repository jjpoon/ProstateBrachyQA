function [result,baselineVal,newVal] = grayscaleTestAuto(imageFile,varargin)
% GRAYSCALETESTAUTO is for the grayscale visibility quality control test.
% The function checks if the length of the gradient strip has changed by
% more than 10% from the baseline measurement.

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
baselineFile = readBaselineFile('Baseline.xls');

% Get baseline value for this test
for i = 1:size(baselineFile,1)
    if ~isempty(strfind(baselineFile{i,1},'Grayscale'))
        baselineVal = baselineFile{i,2};
    end
end

% --- Find top and bottom of visible gradient bar ---
% Crop to gradient scale bar
im_orig = imageFile;
width = size(im_orig,2);
height = size(im_orig,1);
% Crop dimensions
cropX = round(0.75*width);
cropY = round(0.25*height);
cropWidth = round(0.05*width);
cropHeight = round(0.15*height);
im_cropped = im_orig(cropY:cropY+cropHeight,cropX:cropX+cropWidth);

% Store x and y offset for positioning cropped image relative to original image. 
xOffset = cropX - 1;
yOffset = cropY - 1;

% Initialize zero matrix for gradient bar binary image
bar = zeros(size(im_cropped,1),size(im_cropped,2));

% Loop through rows of image, find parts with 15 or more consecutive
% repeating values (parts of the visible gradient bar)
for i = 1:size(im_cropped,1)
    length = 0;
    % Loop through the pixels of each row
    for n = 1:size(im_cropped,2)
        % If pixel is not black and the next pixel has the same intensity value
        if (im_cropped(i,n)>0) && (double(im_cropped(i,n+1)) - double(im_cropped(i,n)) == 0)
            % Store the index of the leftmost part of gradient bar
            if isempty(ind)
                ind = n;
            end
            % Increase length by 1
            length = length + 1;
        else
            % Pixel is black or next pixel has different intensity
            if length < 13
                % If length of repeating values was less than 15, this is
                % not part of the gradient bar. Reset ind and length
                % variables and continue the loop.
                ind = [];
                length = 0;
            end
        end
    end
    % If a section of repeating values was found, set those pixel values to
    % 1
    if ~isempty(ind) && length>=13
        bar(i,ind:ind+length) = 1;
    end
end
% Remove extra lines found (remove regions with area less than 500)
bar = bwareaopen(bar,500);
minimum = zeros(size(bar,1),1);
maximum = zeros(size(bar,1),1);
for r = 1:size(bar,1)
    firstWhite = min(find(bar(r,:)));
    lastWhite = max(find(bar(r,:)));
    if ~isempty(firstWhite)
        minimum(r) = min(find(bar(r,:)));
    end
    if ~isempty(lastWhite)
        maximum(r) = max(find(bar(r,:)));
    end
end
leftBound = mode(minimum(minimum>0));
rightBound = mode(maximum(maximum>0));
bar(:,1:leftBound-1) = 0;
bar(:,rightBound+1:end) = 0;
    

% Get bounding box of gradient bar
region = regionprops(bar,'BoundingBox');
boundingBox = region.BoundingBox;
% Distance from top to bottom of gradient bar in pixels
dist_pixels = boundingBox(4);
% Distance from top to bottom of gradient bar in mm
dist_mm = dist_pixels*pixelScale;
newVal = dist_mm;

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
% Plot top marker
m1 = plot(xOffset+boundingBox(1)+boundingBox(3)/2,yOffset+boundingBox(2),...
    '+','MarkerSize',15,'Linewidth',1.5,'Color','c','Parent',parent);
% Plot bottom marker
m2 = plot(xOffset+boundingBox(1)+boundingBox(3)/2,yOffset+boundingBox(2) + dist_pixels,...
    '+','MarkerSize',15,'Linewidth',1.5,'Color','c','Parent',parent);
% Plot line between markers
line1 = line([get(m1,'XData'),get(m2,'XData')],[get(m1,'YData'),get(m2,'YData')],...
    'LineStyle',':','Color','w','Parent',parent);

% Figure title
% t = title(['Gradient bar length = ' sprintf('%.2f',newVal) ' mm']);
% Legend
l = legend(m1,['Dist: ' sprintf('%.2f',newVal) ' mm'],'Location','southeast');
% Change legend text and background colour
set(l,'TextColor','w','Color',[0.2 0.2 0.2]);

% Callback when double clicking on image
set(im,'ButtonDownFcn',@(obj,eventdata)showInFigure(parent,l));

disp(['Baseline value: ' sprintf('%.2f',baselineVal) ' mm']);
disp(['New value: ' sprintf('%.2f',newVal) ' mm']);

% Change in length of gradient strip
change = abs(newVal - baselineVal);
% Check if length has changed by more than 10%
if isnan(change)
    % Unable to calculate, display error message
    result = [];
elseif change > 0.1*baselineVal
    % Fail
    result = 0;
    disp('Grayscale visbility test: failed');
else
    % Pass
    result = 1;
    disp('Grayscale visbility test: passed');
end

end