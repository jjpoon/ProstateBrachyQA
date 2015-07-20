function [center,widthPoints,heightPoints,ellipseWidth,ellipseHeight] = segmentCrossSection(imageFile)
% SEGMENTCROSSSECTION is used for segmentation of the largest cross section
% in an ultrasound prostate phantom image. The function returns the 
% coordinates of the circle's center, the points marking the width, the
% points marking the height, and the width/height in pixels.

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
[row,col] = find(im_bw);
% Crop tight to objects
im_tight = im_cropped(min(row):max(row),min(col):max(col));

% Store x and y offset for positioning cropped image relative to original image.
xOffset = cropX + min(col) - 2;
yOffset = cropY + min(row) - 2;

% Use adaptive thresholding to create binary image showing the circle
% If no circle found using 100 window size for adaptive thresholding, try
% again using 70 window size
for ws = [100,130,70]
    circle = adaptivethreshold(im_tight,ws,0.001);
    % Morphological operations to improve visibility of circle
    circle = imopen(circle,strel('diamond',5));
    circle = imclose(circle,strel('diamond',5));
    circle = bwareaopen(circle,1000);
    
    % Fill holes
    circle = imfill(circle,'holes');
    % Remove regions connected to border
    circle = imclearborder(circle);
    % Erode twice to smooth object
    % seD = strel('disk',1);
    % circle = imerode(circle,seD);
    % circle = imerode(circle,seD);
    circle = imerode(circle,strel('disk',6));
    % Remove regions with area < 1000
    circle = bwareaopen(circle,1000);
    circle = imdilate(circle,strel('disk',6));
    
    % Get regions from black and white image
    regions = regionprops(circle,'MajorAxisLength','Orientation','EquivDiameter',...
        'Centroid','BoundingBox','Area','Perimeter','Solidity');
    
    % Keep only regions with > 0.9 solidity (circle will have high solidity)
    circleLabel = bwlabel(circle);
    % Find region with highest solidity
    [~,circleInd] = max([regions.Solidity]);
    % Keep region only if solidity is 0.9 or greater
    if ~isempty(circleInd)
        if regions(circleInd).Solidity < 0.9
            circleInd = [];
        end
    end
    circleFinal = ismember(circleLabel,circleInd);
    
    % Find circles using binary image
    circleRegion = regionprops(circleFinal,'BoundingBox','Centroid','Orientation');
    if isempty(circleRegion)
        center = [];
        ellipseWidth = [];
        ellipseHeight = [];
    else
        % Centroid of ellipse
        center = circleRegion.Centroid;
        % Correct with x and y offsets
        center = [center(1)+xOffset, center(2)+yOffset];
        % Width and height
        ellipseWidth = circleRegion.BoundingBox(3);
        ellipseHeight = circleRegion.BoundingBox(4);
        
        % Points for showing width
        widthPoints(1,:) = center - [ellipseWidth/2,0];
        widthPoints(2,:) = center + [ellipseWidth/2,0];
        % Points for showing height
        heightPoints(1,:) = center - [0,ellipseHeight/2];
        heightPoints(2,:) = center + [0,ellipseHeight/2];
    end
    
    % If center was found, continue
    if ~isempty(center)
        break
    end
end

% figure;
% imshow(im_orig);
% hold on
% % Plot markers
% widthMarkers = plot(widthPoints(:,1),widthPoints(:,2),...
%     '+','MarkerSize',10,'Linewidth',2,'Color','r');
% widthLine = line(widthPoints(1:2,1),widthPoints(1:2,2),'LineStyle',':','Color','w');
% heightMarkers = plot(heightPoints(:,1),heightPoints(:,2),...
%     '+','MarkerSize',10,'Linewidth',2,'Color','g');
% heightLine = line(heightPoints(1:2,1),heightPoints(1:2,2),'LineStyle',':','Color','w');

end
