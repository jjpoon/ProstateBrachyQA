function [result,knownVal,measuredVal] = volumeTestAuto(imageFile1,varargin)
% VOLUMETEST is for the volume measurement accuracy quality control test.
% The function checks if the calculated volume is within 5% of the actual
% volume.

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
% Default step size 0.5 cm
addParameter(p,'StepSize',0.5);
addParameter(p,'PanelHandle',[]);
addParameter(p,'AxesHandle',[]);
% Parse inputs
parse(p,imageFile1,varargin{:});
upper = p.Results.UpperScale;
lower = p.Results.LowerScale;
stepSize = p.Results.StepSize;
panelHandle = p.Results.PanelHandle;
axesHandles = p.Results.AxesHandle;

% Get baseline values
if ~exist('Baseline.mat','file')
    % Read xls file if mat file not created yet
    baselineFile = readBaselineFile('Baseline.xls');
else
    % Get baseline value from mat file (faster)
    load('Baseline.mat');
end

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
    
    % Segment circle from image
    [center,radius] = segmentCircle(imageFile);
    
    % Code for plotting the segmented circle on original image
    im_orig = imageFile;
    
    %     if ~isempty(axesHandle)
    %         % Plot on specified axes if given as input
    %         parent = axesHandle;
    %     else
    %         % Otherwise, plot on new figure
    %         parent = gca;
    %     end
    %
    %     % Clear axes first
    %     cla(parent);
    %     % Plot image on axes
    %     imshow(im_orig,'Parent',parent);
    %     % Hold on
    %     set(parent,'NextPlot','add');
    
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
        % Visualize the segmented circle
        c1 = viscircles(parent,center,radius);
        set(c1,'UserData',i);
        
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
        l = [];
        if ~isempty(c1)
            l = legend(c1,['Area: ' sprintf('%.2f',areas(i)) ' cm^2'],...
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
            set([im;c1;l],'Visible','off');
        end
    end
end

% Resize and position figure
if exist('fig','var')
    set(fig,'Units','normalized','Position',[0.05 0.05 0.9 0.85]);
end

% Calculate volume
vol = sum(areas)*stepSize;
measuredVal = vol;

disp(['Areas (cm^2): ' sprintf('%.2f  ',areas)]);
disp(['Step size: ' sprintf('%.2f mm',stepSize*10)]);
disp(['Baseline value: ' sprintf('%.2f',knownVal) ' cm^3']);
disp(['New value: ' sprintf('%.2f',measuredVal) ' cm^3']);

error = abs(measuredVal-knownVal);
% Compare measured volume and known volume
if isempty(error)
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