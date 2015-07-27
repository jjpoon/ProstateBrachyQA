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
% Use decreasing window size if no circle was found.
for ws = 150:-10:70
    for s = 1:2
        circle = adaptivethreshold(im_tight,ws,0.001);
        % Morphological operations to improve visibility of circle
        circle = imopen(circle,strel('disk',s));
        circle = imclose(circle,strel('disk',s));
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
        regions = regionprops(circle,'MajorAxisLength','MinorAxisLength','Orientation',...
            'Centroid','BoundingBox','Area','Perimeter','Solidity');
        
        % Keep only regions with > 0.9 solidity (circle will have high solidity)
        circleLabel = bwlabel(circle);
        % Find region with highest solidity
        [~,circleInd] = max([regions.Solidity]);
        if ~isempty(circleInd)
            % Keep region only if solidity is 0.9 or greater
            if regions(circleInd).Solidity < 0.9
                circleInd = [];
                % Keep region only if major and minor axis length are similar enough
            elseif regions(circleInd).MajorAxisLength > 1.5*regions(circleInd).MinorAxisLength
                circleInd = [];
                % Keep region only if centroid is in reasonable position (avoid
                % false detections)
            elseif abs(regions(circleInd).Centroid(2)/size(im_tight,1) - 0.5) > 0.25 || ...
                    abs(regions(circleInd).Centroid(1)/size(im_tight,2) - 0.5) > 0.25
                circleInd = [];
            end
        end
        circleFinal = ismember(circleLabel,circleInd);
        
        % Find circles using binary image
        circleRegion = regionprops(circleFinal,'BoundingBox','Centroid','Orientation');
        if isempty(circleRegion)
            center = [];
            widthPoints = [];
            heightPoints = [];
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
        
        if ~isempty(center)
            break
        end
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
