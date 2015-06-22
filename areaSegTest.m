imageFile = 'image_no_outline.bmp';

% Crop to ultrasound image
im_orig = imread(imageFile);
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
[row col] = find(im_bw);
% Crop tight to objects
im_tight = im_cropped(min(row):max(row),min(col):max(col));

% Store x and y offset for positioning cropped image relative to original image.
xOffset = cropX + min(col) - 2;
yOffset = cropY + min(row) - 2;

% Filter image using wiener2 (2-D adaptive noise-removal filtering)
filt1 = wiener2(im_tight,[10 10]);
% Filter again using 2D ‘Prewitt’ filter, emphasizing horizontal edges
filt2 = imfilter(filt1,fspecial('prewitt'));

% im_comp = imcomplement(im_tight);
im_open = imopen(im_tight,strel('disk',10));
im_re = imreconstruct(im_open,im_tight);
im_close = imclose(im_re,strel('disk',10));

e = edge(im_close,'Canny',0.1);
im_edge = im_close;
im_edge(e) = 255;

medfilt = medfilt2(im_tight,[30 30]);

im = im_tight;
circle = zeros(size(im,1),size(im,2));
% for r = 1:size(im,1)
%     toggle = 0;
%     for c = 1:size(im,2)-10
%         switch toggle
%             case 0
%                 if im(r,c) > 0
%                     if im(r,c+10) > im(r,c) + 10
%                         circle(r,c) = 1;
%                         toggle = 1;
%                     else
%                         circle(r,c) = 0;
%                     end
%                 end
%             case 1
%                 if im(r,c+10) < im(r,c) - 10
%                     circle(r,c) = 0;
%                     toggle = 0;
%                 else
%                     circle(r,c) = 1;
%                 end
%         end
%     end
% end

for r = 1:size(im,1)
    toggle = 0;
    row = im(r,:);
    avg = mean(row);
    for c = 1:size(im,2)
        if toggle == 1
            if (im(r,c) > im(r,c-1)-4)
                circle(r,c) = 1;
            else
                circle(r,c) = 0;
                toggle = 0;
            end
        else
            if im(r,c) > avg+30
                circle(r,c) = 1;
                toggle = 1;
            else
                circle(r,c) = 0;
            end
        end
    end
end

circle = adaptivethreshold(im_tight,100,0.001);

% circle = im2bw(circle);
circle = imopen(circle,strel('disk',2));
circle = imclose(circle,strel('disk',2));
circle = bwareaopen(circle,1000);
% Remove unwanted white areas from edges of image (scale tick markings, top
% and bottom of ultrasound image)
circle(:,1:30) = 0;         % Remove left edge
circle(:,end-30:end) = 0;   % Remove right edge
circle(1:40,:) = 0;         % Remove top edge
circle(end-120:end,:) = 0;   % Remove bottom edge
imshow(circle);

regions = regionprops(circle,'MajorAxisLength','MinorAxisLength','EquivDiameter','Centroid','BoundingBox','Area','Perimeter');

areas = [regions.Area]';
ind = find(areas>1000);
regions = regions(ind);
perimeters = [regions.Perimeter]';
areas = [regions.Area]';

majorAxisLengths = [regions.MajorAxisLength]';
minorAxisLengths = [regions.MinorAxisLength]';
diff = majorAxisLengths./minorAxisLengths;
mostEqualAxes = abs(diff-1);

circularity = (perimeters.^2)./(4*pi*areas);
mostCircular = abs(circularity-1);

[~,circleRegionInd] = min(mostEqualAxes + mostCircular);
circleRegion = regions(circleRegionInd);

centers = [];
radii = [];
for n = 1:numel(regions)
%     centers = [centers; regions(n).Centroid];
%     radii = [radii; regions(n).EquivDiameter/2];
    rectangle('Position',regions(n).BoundingBox,'EdgeColor','r','LineStyle','--');
    text(regions(n).BoundingBox(1),regions(n).BoundingBox(2),num2str(n),'Color','w');
end
viscircles(centers,radii);

mask = zeros(size(im_tight,1),size(im_tight,2));
x = circleRegion.BoundingBox(1);
y = circleRegion.BoundingBox(2);
width = circleRegion.BoundingBox(3);
height = circleRegion.BoundingBox(4);
mask(y:y+height,x:x+width) = 1;
mask = imdilate(mask,ones(10));

im_circle = im_tight.*uint8(mask);
im_circle = imclose(im_circle,strel('disk',10));

% imshow(im_tight);
% viscircles(circleRegion.Centroid,circleRegion.EquivDiameter/2);
% 
% [accum, circen, cirrad] = CircularHough_Grd(im_tight, [90 120], 3, 3, 1);
% 
% figure; imshow(im_tight);
% hold on
% for n = 1:size(circen,1)
%     radius = cirrad(n);
%     if radius > 0
%         centerX = circen(n,1);
%         centerY = circen(n,2);
%         rectangle('Position',[centerX - radius, centerY - radius, radius*2, radius*2],...
%             'Curvature',[1,1],'EdgeColor','b');
%     end
% end

% close = imclose(circle,strel('disk',30));
% % [accum, centers, radii] = CircularHough_Grd(close, [5 110], 20, 20, 1);

% thresh=thresholdLocally(im_tight,[5 5]);

disp(circularity(circleRegionInd));
if circularity(circleRegionInd) > 3.5
    centers = [];
    radii = [];
else
    radius = circleRegion.MajorAxisLength/2;
    radiusRange = [round(0.75*radius) round(1.25*radius)];
    sensitivity = 0.96;
    edgethresh = 0.04;
    while isempty(centers)
        [centers,radii] = imfindcircles(im_circle,radiusRange,'Sensitivity',sensitivity,...
            'EdgeThreshold',edgethresh,'ObjectPolarity','bright');
        if isempty(centers)
            sensitivity = sensitivity + 0.01;
        end
    end
end
imshow(im_tight)
hold on
viscircles(centers,radii)

% a = adaptivethreshold(im_tight,100,0.001);
% 
% [centers,radii] = imfindcircles(a,[30 50],'Sensitivity',0.96,'EdgeThreshold',0.4);
% imshow(im_tight)
% hold on
% viscircles(centers,radii)
