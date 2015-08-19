function [center,lengthPoints,ellipseLength] = segmentEllipse(imageFile)
% SEGMENTELLIPSE is used for segmentation of an ellipse in an ultrasound
% prostate phantom image. The function returns the coordinates of the
% ellipse's center, the left and right point of the ellipse showing the
% length, and the major axis length.

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
% Use decreasing window size if no ellipse was found.
for ws = 70:10:170
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
        regions = regionprops(circle,'MajorAxisLength','MinorAxisLength',...
            'Orientation','Centroid','BoundingBox','Area','Solidity');
        
        % Keep only regions with > 0.9 solidity (circle will have high solidity)
        circleLabel = bwlabel(circle);
        % Find region with highest solidity
        [~,circleInd] = max([regions.Solidity]);
        if ~isempty(circleInd)
            if regions(circleInd).Solidity < 0.9
                % Keep region only if solidity is 0.9 or greater
                circleInd = [];
            elseif regions(circleInd).MajorAxisLength > 0.9*max(size(im_tight))
               % Keep region if not unrealistically large
               circleInd = [];
            elseif regions(circleInd).MinorAxisLength < 0.1*max(size(im_tight))
               % Keep region if not unrealistically large
               circleInd = [];
            else
                % Keep region if in reasonable position
                xPos = regions(circleInd).Centroid(1)/size(im_tight,2);
                yPos = regions(circleInd).Centroid(2)/size(im_tight,1);
                if xPos > 0.85 || xPos < 0.15 || yPos > 0.75 || yPos < 0.25
                    circleInd = [];
                end
            end
        end
        circleFinal = ismember(circleLabel,circleInd);
        
        % Find circles using binary image
        circleRegion = regionprops(circleFinal,'MajorAxisLength','Centroid','Orientation');
        if isempty(circleRegion)
            center = [];
            ellipseLength = [];
            lengthPoints = [];
        else
            % Centroid of ellipse
            center = circleRegion.Centroid;
            % Correct with x and y offsets
            center = [center(1)+xOffset, center(2)+yOffset];
            % Angle between x-axis and major axis of region (neg. because MATLAB y
            % increases going down)
            theta = -(circleRegion.Orientation);
            % Length of major axis
            ellipseLength = circleRegion.MajorAxisLength;
            % Rotation matrix to get vector pointing along major axis
            R = [cosd(theta) -sind(theta); sind(theta) cosd(theta)];
            % Vector pointing along major axis
            v = R*[1;0];
            
            % Points for showing lateral resolution
            lengthPoints(1,:) = center + (ellipseLength/2)*v';
            lengthPoints(2,:) = center - (ellipseLength/2)*v';
        end
        
        % Check for false detection by making sure average intensity within
        % ellipse is higher than average intensity of pixels outside ellipse.
        if ~isempty(center)
            insideMask = circleFinal;
            insideCircle = im_tight.*uint8(insideMask);
            outsideMask = imdilate(circleFinal,strel('disk',20)) - circleFinal;
            outsideCircle = im_tight.*uint8(outsideMask);
            meanInside = mean(insideCircle(insideCircle>0));
            meanOutside = mean(outsideCircle(outsideCircle>0));
            if meanInside/meanOutside < 1.15
                center = [];
                ellipseLength = [];
                lengthPoints = [];
            end
        end
        
        % If center was found, continue
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
% m = plot(lengthPoints(:,1),lengthPoints(:,2),...
%     '+','MarkerSize',10,'Linewidth',2,'Color','c');

end
