function [center,radius] = segmentCircle(imageFile)
% SEGMENTCIRCLE is used for segmentation of a circle in an ultrasound
% prostate phantom image. The function returns the coordinates of the
% circle's center and the radius in pixels.

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
for ws = 170:-10:70
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
        circle = imclose(circle,strel('disk',30));
        
        % Get regions from black and white image
        regions = regionprops(circle,'MajorAxisLength','MinorAxisLength','EquivDiameter',...
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
            else
                % Keep region if in reasonable position
                xPos = regions(circleInd).Centroid(1)/size(im_tight,2);
                yPos = regions(circleInd).Centroid(2)/size(im_tight,1);
                if xPos > 0.75 || xPos < 0.25 || yPos > 0.75 || yPos < 0.25
                    circleInd = [];
                end
            end
        end
        circleFinal = ismember(circleLabel,circleInd);
        
        % Find circles using binary image
        circleRegion = regionprops(circleFinal,'Centroid','MajorAxisLength');
        if isempty(circleRegion)
            center = [];
            radius = [];
        else
            radius = circleRegion.MajorAxisLength/2;
            radiusRange = [round(0.75*radius) round(1.25*radius)];
            [centers,radii] = imfindcircles(circleFinal,radiusRange,'Sensitivity',0.99,...
                'EdgeThreshold',0.04,'ObjectPolarity','bright');
            % Get circle with center closest to region centroid with radius
            % closest to region MajorAxisLength/2
            centroid = repmat(circleRegion.Centroid,size(centers,1),1);
            centerDiff = abs(sqrt((centers(:,1)-centroid(:,1)).^2 + (centers(:,2)-centroid(:,2)).^2));
            radiusDiff = abs(radii - circleRegion.MajorAxisLength/2);
            [~,circleInd] = min(centerDiff + radiusDiff);
            center = centers(circleInd,:);
            radius = radii(circleInd);
        end
        
        % Check for false detection by making sure average intensity within
        % circle is higher than average intensity of pixels outside circle.
        if ~isempty(center)
            insideMask = circleFinal;
            insideCircle = im_tight.*uint8(insideMask);
            outsideMask = imdilate(circleFinal,strel('disk',20)) - circleFinal;
            outsideCircle = im_tight.*uint8(outsideMask);
            meanInside = mean(insideCircle(insideCircle>0));
            meanOutside = mean(outsideCircle(outsideCircle>0));
            if meanInside - meanOutside < 10
                center = [];
                radius = [];
            end
        end
        
        if ~isempty(center)
            break
        end
    end
    
    if ~isempty(center)
        break
    end
end

% % Code for plotting the segmented circle on cropped image
% imshow(im_tight);
% hold on
% viscircles(center,radius);

% % Code for plotting the segmented shape outline on cropped image
% outline = bwperim(circleFinal);
% im_outline = im_tight;
% im_outline(outline) = 255;
% imshow(im_outline);

% Correct center with x and y offset and plot on original image
if ~isempty(center)
    center(1) = center(1) + xOffset;
    center(2) = center(2) + yOffset;
end

% % Code for plotting the segmented circle on original image
% imshow(im_orig);
% hold on
% viscircles(center,radius);

end
