function [result,knownVal,measuredVal,freq] = volumeTestFormula(axialImage,sagittalImage,varargin)
% VOLUMETEST is for the volume measurement accuracy quality control test.
% The function checks if the calculated volume is within 5% of the actual
% volume.

% Input parser
p = inputParser;
% Require axial view image of largest cross section for height/width
addRequired(p,'axialImage',@isnumeric);
% Require sagittal view image for length measurement
addRequired(p,'sagittalImage',@isnumeric);
addParameter(p,'UpperScaleAxial',[]);
addParameter(p,'LowerScaleAxial',[]);
addParameter(p,'UpperScaleSagittal',[]);
addParameter(p,'LowerScaleSagittal',[]);
% Default step size 5 mm
addParameter(p,'StepSize',5);
addParameter(p,'PanelHandle',[]);
addParameter(p,'AxesHandle',[]);
addParameter(p,'EllipsoidAxes',[]);
% Parse inputs
parse(p,axialImage,sagittalImage,varargin{:});
upperAxial = p.Results.UpperScaleAxial;
lowerAxial = p.Results.LowerScaleAxial;
upperSagittal = p.Results.UpperScaleSagittal;
lowerSagittal = p.Results.LowerScaleSagittal;
panelHandle = p.Results.PanelHandle;
axesHandles = p.Results.AxesHandle;
ellipsoidAxes = p.Results.EllipsoidAxes;

% Read frequency
freq = readFrequency(axialImage);

% Get baseline values
baselineFile = readBaselineFile('Baseline.xls');

% Get baseline value for this test
for i = 1:size(baselineFile,1)
    if ~isempty(strfind(baselineFile{i,1},'Volume'))
        knownVal = baselineFile{i,2};
    end
end

% If running function without GUI, plot on new figure
if isempty(panelHandle) && isempty(axesHandles)
    fig = figure;
end

% Clear axes first
% clf(parent);

%% Axial view
if isempty(upperAxial) && isempty(lowerAxial)
    % Get the pixel to mm conversion ratio, automatically reading scale
    % labels from the image
    pixelScale = getPixelScale(axialImage);
else
    % Get pixel scale using scale readings inputted by user
    pixelScale = getPixelScale(axialImage,upperAxial,lowerAxial);
end

% Segment cross section from image
[center,widthPoints,heightPoints,ellipseWidth,ellipseHeight] = segmentCrossSection(axialImage);
width_mm = ellipseWidth*pixelScale;
height_mm = ellipseHeight*pixelScale;

% Code for plotting the segmented circle on original image
im_orig = axialImage;

for ii = 1:2
    
    if ii == 1
        % Set size of image grid based on number of inputted images
        gridSize = [1,2];
        n = gridSize(1);
        lengthMarkers = gridSize(2);
        [c,r] = ind2sub([gridSize(2) gridSize(1)], 1);
        % Create subplots
        if ~isempty(panelHandle)
            parent = subplot('Position', [(c-1)/lengthMarkers, 1-(r)/n, 1/lengthMarkers, 1/n],'Parent',panelHandle);
        else
            parent = subplot('Position', [(c-1)/lengthMarkers, 1-(r)/n, 1/lengthMarkers, 1/n],'Parent',fig);
        end
    else
        if ~isempty(axesHandles)
            parent = axesHandles(1);
        else
            % Running without GUI, break out of this second loop
            break
        end
    end
    
    % Plot image on axes
    im = imshow(im_orig,'Parent',parent);
    set(im,'UserData',1);
    % Hold on
    set(parent,'NextPlot','add');
    % Plot markers
    widthMarkers = plot(widthPoints(:,1),widthPoints(:,2),...
        '+','MarkerSize',10,'Linewidth',1.5,'Color','r','Parent',parent);
    widthLine = line(widthPoints(1:2,1),widthPoints(1:2,2),'LineStyle',':',...
        'Color','w','Parent',parent);
    heightMarkers = plot(heightPoints(:,1),heightPoints(:,2),...
        '+','MarkerSize',10,'Linewidth',1.5,'Color','g','Parent',parent);
    heightLine = line(heightPoints(1:2,1),heightPoints(1:2,2),'LineStyle',':',...
        'Color','w','Parent',parent);
    
    % Legend
    l = legend([widthMarkers,heightMarkers],...
        ['Width: ' sprintf('%.2f',width_mm) ' mm'],...
        ['Height: ' sprintf('%.2f',height_mm) ' mm'],...
        'Location','southeast','Orientation','vertical');
    % Change legend text and background colour
    set(l,'TextColor','w','Color',[0.2 0.2 0.2]);
    % Add image index to UserData, used for finding legend
    % associated with image
    userData = get(l,'UserData');
    userData.ImageIndex = 1;
    set(l,'UserData',userData);
    
    % Callback when double clicking on image
    set(im,'ButtonDownFcn',@(obj,eventdata)showInFigure(parent,l));
    
    if ii == 2
        % Hide axes plots, GUI function will show the current axes only
        set([im;widthMarkers;heightMarkers;widthLine;heightLine;l],'Visible','off');
    end
end
%% Sagittal view
if isempty(upperSagittal) && isempty(lowerSagittal)
    % Get the pixel to mm conversion ratio, automatically reading scale
    % labels from the image
    pixelScale = getPixelScale(sagittalImage);
else
    % Get pixel scale using scale readings inputted by user
    pixelScale = getPixelScale(sagittalImage,upperSagittal,lowerSagittal);
end

% Segment ellipse from image
[center,lengthPoints,ellipseLength] = segmentEllipse(sagittalImage);
length_mm = ellipseLength*pixelScale;

% Code for plotting the segmented circle on original image
im_orig = sagittalImage;

for ii = 1:2
    
    if ii == 1
        % Set size of image grid based on number of inputted images
        gridSize = [1,2];
        n = gridSize(1);
        lengthMarkers = gridSize(2);
        [c,r] = ind2sub([gridSize(2) gridSize(1)], 2);
        % Create subplots
        if ~isempty(panelHandle)
            parent = subplot('Position', [(c-1)/lengthMarkers, 1-(r)/n, 1/lengthMarkers, 1/n],'Parent',panelHandle);
        else
            parent = subplot('Position', [(c-1)/lengthMarkers, 1-(r)/n, 1/lengthMarkers, 1/n],'Parent',fig);
        end
    else
        if ~isempty(axesHandles)
            parent = axesHandles(2);
        else
            % Running without GUI, break out of this second loop
            break
        end
    end
    
    % Plot image on axes
    im = imshow(im_orig,'Parent',parent);
    set(im,'UserData',2);
    % Hold on
    set(parent,'NextPlot','add');
    % Plot markers
    lengthMarkers = plot(lengthPoints(:,1),lengthPoints(:,2),...
        '+','MarkerSize',10,'Linewidth',1.5,'Color','c','Parent',parent);
    lengthLine = line(lengthPoints(1:2,1),lengthPoints(1:2,2),...
        'LineStyle',':','Color','w','Parent',parent);
    
    % Legend
    l = legend(lengthMarkers,['Length: ' sprintf('%.2f',length_mm) ' mm'],...
        'Location','southeast','Orientation','horizontal');
    % Change legend text and background colour
    set(l,'TextColor','w','Color',[0.2 0.2 0.2]);
    % Add image index to UserData, used for finding legend
    % associated with image
    userData = get(l,'UserData');
    userData.ImageIndex = 2;
    set(l,'UserData',userData);
    
    % Callback when double clicking on image
    set(im,'ButtonDownFcn',@(obj,eventdata)showInFigure(parent,l));
    
    if ii == 2
        % Hide axes plots, GUI function will show the current axes only
        set([im;lengthMarkers;lengthLine;l],'Visible','off');
    end
end
%% Calculate volume

% Prostate ellipsoid formula
width_cm = width_mm/10;
height_cm = height_mm/10;
length_cm = length_mm/10;
vol =  width_cm * height_cm * length_cm * (pi/6);
measuredVal = vol;

% Plot volume slices 3D visualization
if ~isempty(ellipsoidAxes)
    % Hold on
    set(ellipsoidAxes,'NextPlot','add');
    
    yPos = 0;
    zPos = 0;
    % Get x,y,z coordinates of contour for vertical cross section
    th = 0:pi/50:2*pi;
    yunit = width_mm/2 * cos(th) + yPos;
    zunit = height_mm/2 * sin(th) + zPos;
    xunit = zeros(size(yunit));
    % Plot contour line
    plot3(xunit,yunit,zunit,'b','Parent',ellipsoidAxes,'Parent',ellipsoidAxes);

%     % Plot width
%     plot3([0,0],[-width_mm/2,width_mm/2],[0,0],'r','Parent',ellipsoidAxes);
%     % Plot height
%     plot3([0,0],[0,0],[-height_mm/2,height_mm/2],'g','Parent',ellipsoidAxes);
%     % Plot length
%     plot3([-length_mm/2,length_mm/2],[0,0],[0,0],'c','Parent',ellipsoidAxes);

    % Plot ellipsoid
    [x,y,z] = ellipsoid(ellipsoidAxes,0,0,0,length_mm/2,width_mm/2,height_mm/2);
    surf(x,y,z,'EdgeColor','none','FaceColor','b','Parent',ellipsoidAxes);
    % Set lighting and camera properties
    fig1 = findobj(get(0,'Children'),'Name','ProstateBrachyQA');
    set(fig1,'CurrentAxes',ellipsoidAxes);
    camlight;
    lighting gouraud;
    view(3);
    axis off;
    axis equal;
end

% Resize and position figure
if exist('fig','var')
    set(fig,'Units','normalized','Position',[0.05 0.05 0.9 0.85]);
end

disp('Volume formula: width * height * length * pi/6');
disp(['Width: ' sprintf('%.2f  ',width_mm)]);
disp(['Height: ' sprintf('%.2f  ',height_mm)]);
disp(['Length: ' sprintf('%.2f  ',length_mm)]);
disp(['Baseline value: ' sprintf('%.2f',knownVal) ' cm^3']);
disp(['New value: ' sprintf('%.2f',measuredVal) ' cm^3']);

error = abs(measuredVal-knownVal);
% Compare measured volume and known volume
if isnan(error)
    result = [];
    disp('Missing information - could not complete test');
elseif error > 0.05*knownVal
    % Fail
    result = 0;
    disp('Volume measurement accuracy test: failed');
else
    % Pass
    result = 1;
    disp('Volume measurement accuracy test: passed');
end

end