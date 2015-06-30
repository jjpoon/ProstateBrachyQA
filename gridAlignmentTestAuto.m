function [result,errors] = gridAlignmentTestAuto(imageFile1,varargin)
% GRIDALIGNMENTEST is for the needle template alignment quality control test.
% The function checks the difference between the actual needle location 
% (using the needle template) and the corresponding point on the electronic
% grid overlay. The four corners and the center of the grid are tested. 
% Alignment must be correct to within 3 mm.

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
addParameter(p,'PanelHandle',[]);
addParameter(p,'AxesHandle',[]);
% Parse inputs
parse(p,imageFile1,varargin{:});
upper = p.Results.UpperScale;
lower = p.Results.LowerScale;
panelHandle = p.Results.PanelHandle;
axesHandles = p.Results.AxesHandle;

% If running function without GUI, plot on new figure
if isempty(panelHandle) && isempty(axesHandles)
    fig = figure;
end

% Clear axes first
% clf(parent);

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
        
    % Read image
    im_orig = imread(imageFile);
    width = size(im_orig,2);
    height = size(im_orig,1);
    % Crop dimensions
    cropX = round(0.15*width);
    cropY = round(0.2*height);
    cropWidth = round(0.7*width);
    cropHeight = round(0.65*height);
    im_cropped = im_orig(cropY:cropY+cropHeight,cropX:cropX+cropWidth,:);
    % Make all non-black objects white
    im_bw = im2bw(im_cropped,0.01);
    % Get indices of rows/columns with nonzero elements
    [row col] = find(im_bw);
    % Crop tight to objects
    im_tight = im_cropped(min(row):max(row),min(col):max(col),:);
    
    % Store x and y offset for positioning cropped image relative to original image.
    xOffset = cropX + min(col) - 2;
    yOffset = cropY + min(row) - 2;
    
    % ---------------------------------------------------------------------
    % Get template grid points
    
    % Get image with only yellow pixels
    pixelMask = im_tight(:,:,1) == 255 & im_tight(:,:,2) == 255 & im_tight(:,:,3) == 0;
    im_grid_yellow = rgb2gray(im_tight).*uint8(pixelMask);
    % Convert to black and white
    im_grid_yellow = im2bw(im_grid_yellow);
    % Remove large objects
    im_grid_yellow = im_grid_yellow - bwareaopen(im_grid_yellow,6);
    
    % Get image with only cyan pixels
    pixelMask = im_tight(:,:,1) == 128 & im_tight(:,:,2) == 255 & im_tight(:,:,3) == 255;
    im_grid_cyan = rgb2gray(im_tight).*uint8(pixelMask);
    % Convert to black and white
    im_grid_cyan = im2bw(im_grid_cyan);
    % Remove large objects
%     im_grid_cyan = im_grid_cyan - bwareaopen(im_grid_cyan,6);
    
    % Final grid template
    im_grid = im_grid_yellow + im_grid_cyan;
    % Convert to black and white
    im_grid = im2bw(im_grid);
%     % Remove large objects
%     im_grid = im_grid - bwareaopen(im_grid,6);
%     % Remove small objects
%     im_grid = bwareaopen(im_grid,5);
    
    % Get grid point regions
    gridRegions = regionprops(im_grid,'Centroid');
    % Get centroids of grid point regions
    for n = 1:numel(gridRegions)
        centroids(n,:) = gridRegions(n).Centroid;
    end
    
    % ---------------------------------------------------------------------
    % Get the indices of the corner points and center
    
    % Top left
    topLeftCorner = [0 0];
    % Get distances from corner to centroids
    vectors = centroids - repmat(topLeftCorner,size(centroids,1),1);
    distances = sqrt(vectors(:,1).^2 + vectors(:,2).^2);
    [topLeft,topLeftInd] = min(distances);
    
    % Top right
    topRightCorner = [size(im_grid,2) 0];
    % Get distances from corner to centroids
    vectors = centroids - repmat(topRightCorner,size(centroids,1),1);
    distances = sqrt(vectors(:,1).^2 + vectors(:,2).^2);
    [topRight,topRightInd] = min(distances);
    
    % Bottom left
    bottomLeftCorner = [0 size(im_grid,1)];
    % Get distances from corner to centroids
    vectors = centroids - repmat(bottomLeftCorner,size(centroids,1),1);
    distances = sqrt(vectors(:,1).^2 + vectors(:,2).^2);
    [bottomLeft,bottomLeftInd] = min(distances);
    
    % Top right
    bottomRightCorner = [size(im_grid,2) size(im_grid,1)];
    % Get distances from corner to centroids
    vectors = centroids - repmat(bottomRightCorner,size(centroids,1),1);
    distances = sqrt(vectors(:,1).^2 + vectors(:,2).^2);
    [bottomRight,bottomRightInd] = min(distances);
    
    % Center
    gridHeight_v = centroids(bottomLeftInd,:) - centroids(topLeftInd,:);
    gridWidth_v = centroids(topRightInd,:) - centroids(topLeftInd,:);
    center = centroids(topLeftInd,:) + gridHeight_v/2 + gridWidth_v/2;
    % Get distances from center to centroids
    vectors = centroids - repmat(center,size(centroids,1),1);
    distances = sqrt(vectors(:,1).^2 + vectors(:,2).^2);
    [center,centerInd] = min(distances);
    
    gridIndices = [topLeftInd,topRightInd,bottomLeftInd,bottomRightInd,centerInd];
    % ---------------------------------------------------------------------
    
%     % Show corner and center points
%     imshow(ismember(bwlabel(im_grid),gridIndices));
    
    % ---------------------------------------------------------------------
    % Compare region intensities (find point where needle is)
    regionWidth = 100;
    regionHeight = 100;
    for n = gridIndices
        point = centroids(n,:);
        % Get region around current grid point
        region{n} = im_tight(round(point(2)-regionHeight/2:point(2)+regionHeight/2),...
            round(point(1)-regionWidth/2:point(1)+regionWidth/2),:);
        % Convert to grayscale
        region_gray{n} = rgb2gray(region{n});
        % Measure total intensity by summing all pixels
        regionIntensity(n) = sum(region_gray{n}(:));
    end
    % Get index of region with greatest intensity
    [~,brightestInd] = max(regionIntensity);
    % Get the grid point with brightest surroundings
    gridPoint = gridRegions(brightestInd).Centroid;
    
    % Region mask
    regionMask = zeros(size(im_tight,1),size(im_tight,2));
    regionMask(round(gridPoint(2)-regionHeight/2:gridPoint(2)+regionHeight/2),...
        round(gridPoint(1)-regionWidth/2:gridPoint(1)+regionWidth/2)) = 1;
    % Restrict image to region of interest
    reg = rgb2gray(im_tight).*uint8(regionMask);
    regBW = im2bw(reg);
    bwRegions = regionprops(regBW,'Area','Centroid','MinorAxisLength','Orientation');
    
    [~,biggest] = max([bwRegions.Area]);
    needleRegion = bwRegions(biggest);
    
    % Centroid of needle region
    centroid = needleRegion.Centroid;
%     % Angle between x-axis and minor axis of region (neg. because MATLAB y
%     % increases going down)
%     theta = -(needleRegion.Orientation+90);
%     % Length of minor axis
%     minorAxisLength = needleRegion.MinorAxisLength;
%     % Rotation matrix to get vector pointing along minor axis
%     R = [cosd(theta) -sind(theta); sind(theta) cosd(theta)];
%     % Vector pointing along minor axis
%     v = R*[1;0];
%     
%     % Needle point
%     needlePoint = centroid - (minorAxisLength/2)*v';
    needlePoint = centroid;
    
    % Modify needle point and grid point by x and y offset so that they are
    % plotted in correct position with respect to original image
    gridPoint = [gridPoint(1)+xOffset, gridPoint(2)+yOffset];
    needlePoint = [needlePoint(1)+xOffset, needlePoint(2)+yOffset];
    
    % ---------------------------------------------------------------------
    % Plot markers on figure to show grid and needle points that were found
    % Create column vectors for offsets, used for plotting markers
    numImages = numel(imageInputs);
    
    for ii = 1:2
        
        if ii == 1
            % Set size of image grid based on number of inputted images
            gridSize = [floor(sqrt(numImages)),ceil(sqrt(numImages))];
            n = gridSize(1);
            m = gridSize(2);
            [c,r] = ind2sub([gridSize(2) gridSize(1)], i);
            % Create subplots
            if ~isempty(panelHandle)
                parent = subplot('Position', [(c-1)/m, 1-(r)/n, 1/m, 1/n],'Parent',panelHandle);
            else
                parent = subplot('Position', [(c-1)/m, 1-(r)/n, 1/m, 1/n],'Parent',fig);
            end
        else
            if ~isempty(axesHandles)
                parent = axesHandles(i);
            else
                % Running without GUI, break out of this second loop
                break
            end
        end
        
        % Plot image on axes
        im = imshow(im_orig,'Parent',parent);
        set(im,'UserData',i);
        % Hold on
        set(parent,'NextPlot','add');
        % Plot markers
        gridMarker = plot(gridPoint(1),gridPoint(2),'+','MarkerSize',10,...
            'LineWidth',2,'Color','r','Parent',parent);
        needleMarker = plot(needlePoint(1),needlePoint(2),'+','MarkerSize',10,...
            'LineWidth',2,'Color','c','Parent',parent);
        % Plot distance line
        line1 = line([gridPoint(1),needlePoint(1)],[gridPoint(2),needlePoint(2)],...
            'LineStyle','--','Parent',parent);
        
        % Get distance between grid and needle point
        dist_pixels = norm(needlePoint - gridPoint);
        dist_mm = dist_pixels*pixelScale;
        
        % Figure title
        %     title(['Error = ' sprintf('%.2f',dist_mm) ' mm']);
        % Legend
        l = [];
        if ~isempty(needleMarker)
            l = legend(needleMarker,['Dist: ' sprintf('%.2f',dist_mm) ' mm'],...
                'Location','southeast','Orientation','horizontal');
            % Decrease legend marker size
            markerObjs = findobj(get(l,'children'), 'type', 'line');
            set(markerObjs, 'Markersize', 12);
            % Change legend text and background colour
            set(l,'TextColor','w','Color',[0.2 0.2 0.2]);
            % Add image index to UserData, used for finding legend
            % associated with image
            userData = get(l,'UserData');
            userData.ImageIndex = i;
            set(l,'UserData',userData);
        end
        
        % Callback when double clicking on image
        set(im,'ButtonDownFcn',@(obj,eventdata)showInFigure(parent,l));
        
        if ii == 2
            % Hide axes plots, GUI function will show the current axes only
            set([im;gridMarker;needleMarker;line1;l],'Visible','off');
        end
        
    end
    % ---------------------------------------------------------------------
    
    % Error between grid point and needle point
    errors(i) = dist_mm;
    
    % Result
    if dist_mm > 3
        result(i) = 0;
    else
        result(i) = 1;
    end
    
end

% Resize and position figure
if exist('fig','var')
    set(fig,'Units','normalized','Position',[0.05 0.05 0.9 0.85]);
end

disp(['Errors (mm): ' sprintf('%.2f  ',errors)]);

% Compare errors between needle template and electronic grid
% Check if any errors (corners and center) are greater than 3 mm.
if any(errors>3)
    % Fail
    disp('Needle template alignment test: failed');
else
    % Pass
    disp('Needle template alignment test: passed');
end

end