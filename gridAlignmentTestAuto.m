function [result,errors,probeToG2] = gridAlignmentTestAuto(imageFile1,varargin)
% GRIDALIGNMENTESTAUTO is for the needle template alignment quality control test.
% The function checks the difference between the actual needle location 
% (using the needle template) and the corresponding point on the electronic
% grid overlay. The four corners and the center of the grid are tested. 
% Alignment must be correct to within 3 mm.

% Input parser
p = inputParser;
% Require at least one image
addRequired(p,'imageFile1',@isnumeric);
% Get additional image inputs
imageInputs = {};
for i = 1:numel(varargin)
    if isnumeric(varargin{i})
        if all(size(varargin{i}) > 1)
            imageInputs{end+1} = varargin{i};
        end
    end
end
% Add the first required image filename to list of images
imageInputs = [{imageFile1},imageInputs];
% Add enough optional inputs for the number of images given
for n = 2:numel(imageInputs)
    addOptional(p,['imageFile' num2str(n)],[],@isnumeric);
end
addParameter(p,'UpperScale',[]);
addParameter(p,'LowerScale',[]);
addParameter(p,'PanelHandle',[]);
addParameter(p,'AxesHandle',[]);
addParameter(p,'GridCoords',[]);
% Parse inputs
parse(p,imageFile1,varargin{:});
upperScale = p.Results.UpperScale;
lowerScale = p.Results.LowerScale;
panelHandle = p.Results.PanelHandle;
axesHandles = p.Results.AxesHandle;
gridCoords = p.Results.GridCoords;

% If running function without GUI, plot on new figure
if isempty(panelHandle) && isempty(axesHandles)
    fig = figure;
end

% Clear axes first
% clf(parent);

% Load images and read labels
for i = 1:numel(imageInputs)
    imageFile = imageInputs{i};
    
    if isempty(upperScale) && isempty(lowerScale)
        % Get the pixel to mm conversion ratio, automatically reading scale
        % labels from the image
        pixelScale = getPixelScale(imageFile);
    else
        % Get pixel scale using scale readings inputted by user
        pixelScale = getPixelScale(imageFile,upperScale,lowerScale);
    end
        
    % Read image
    im_orig = imageFile;
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
    pixelMask = im_tight(:,:,1) > 200 & im_tight(:,:,2) > 200 & im_tight(:,:,3) < 100;
    im_grid_yellow = rgb2gray(im_tight).*uint8(pixelMask);
    % Convert to black and white
    im_grid_yellow = im2bw(im_grid_yellow);
    % Remove large objects
    im_grid_yellow = im_grid_yellow - bwareaopen(im_grid_yellow,6);
    
%     % Get image with only cyan pixels
%     pixelMask = im_tight(:,:,1) == 128 & im_tight(:,:,2) == 255 & im_tight(:,:,3) == 255;
%     im_grid_cyan = rgb2gray(im_tight).*uint8(pixelMask);
%     % Convert to black and white
%     im_grid_cyan = im2bw(im_grid_cyan);
%     % Remove large objects
%     im_grid_cyan = im_grid_cyan - bwareaopen(im_grid_cyan,6);
    
    % Final grid template
    im_grid = im_grid_yellow;
%     im_grid = im_grid_yellow + im_grid_cyan;
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
    
    % Get size of grid template
    rows = numel(find(centroids(:,1)==max(centroids(:,1))));
    cols = numel(find(centroids(:,2)==max(centroids(:,2))));
    % Convert grid coordinates to row and column number
    colNum = double(upper(gridCoords{i}(1)))-64;
    % Grid template row coordinates increase going up
    r = rows:-1:1;
    rowNum = r(str2double(gridCoords{i}(2:end)));
    % Get index of grid point from coordinates
    gridInd = sub2ind([rows cols],rowNum,colNum);
    gridPoint = gridRegions(gridInd).Centroid;
    
    % Region mask
    regionWidth = 100;
    regionHeight = 100;
    % Size of image
    imageHeight = size(im_tight,1);
    imageWidth = size(im_tight,2);
    % Define borders of region, making sure they are within the image (not
    % negative or greater than image width/height)
    top = max(1,round(gridPoint(2)-regionHeight/2));
    bottom = min(imageHeight,round(gridPoint(2)+regionHeight/2));
    left = max(1,round(gridPoint(1)-regionWidth/2));
    right = min(imageWidth,round(gridPoint(1)+regionWidth/2));
    % Create region mask
    regionMask = zeros(imageHeight,imageWidth);
    regionMask(top:bottom,left:right) = 1;
    % Restrict image to region of interest
    reg = rgb2gray(im_tight).*uint8(regionMask);
    % Remove grid template from image
    reg = reg.*uint8(imcomplement(pixelMask));
    % Find brightest region (needle point)
    for thresh = 0.9:-0.1:0.5
        regBW = im2bw(reg,thresh);
        if max(regBW(:)) > 0
            break
        end
    end
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
            gridSize = [round(sqrt(numImages)),ceil(sqrt(numImages))];
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
            'LineWidth',1.5,'Color','r','Parent',parent);
        needleMarker = plot(needlePoint(1),needlePoint(2),'+','MarkerSize',10,...
            'LineWidth',1.5,'Color','c','Parent',parent);
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

% Get probe to G2 distance (using same grid  template and pixel scale as images)
% Convert G2 grid coordinate to row and column number
colNum = double('G')-64;
% Grid template row coordinates increase going up
r = rows:-1:1;
rowNum = r(2);
% Get index of grid point from coordinates
G2Ind = sub2ind([rows cols],rowNum,colNum);
G2Point = gridRegions(G2Ind).Centroid;
% Get probe point (top of semi-circle)
im_bw = im2bw(im_tight,0.01);
im_bw = imcomplement(im_bw);
% Remove large regions
im_bw = im_bw - bwareaopen(im_bw,10000);
% Remove small regions
im_bw = bwareaopen(im_bw,100);
% Limit to probe region
im_bw(1:round(0.9*size(im_bw,1)),:) = 0;
im_bw(:,1:round(0.35*size(im_bw,2))) = 0;
im_bw(:,round(0.65*size(im_bw,2)):end) = 0;
% Get y position of probe surface
y = find(im_bw(:,round(size(im_bw,2)/2)),1,'first');
% If top of probe could not be found (eg. image gain too low), assume y = 540
if isempty(y)
    y = 540;
end
probePoint = [size(im_bw,2)/2,y];
% Get probe to G2 distance
probeToG2_pixels = norm(G2Point - probePoint);
probeToG2 = probeToG2_pixels*pixelScale;

% Resize and position figure
if exist('fig','var')
    set(fig,'Units','normalized','Position',[0.05 0.05 0.9 0.85]);
end

disp(['Errors (mm): ' sprintf('%.2f  ',errors)]);
disp(['Probe to G2 (mm): ' sprintf('%.2f',probeToG2)]);

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