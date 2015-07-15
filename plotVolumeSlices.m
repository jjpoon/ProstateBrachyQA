function plotVolumeSlices(radii,varargin)

% Input parser
p = inputParser;
% Require at least one image
addRequired(p,'radii',@isnumeric);
addParameter(p,'AxesHandle',[]);
% Default step size 5 mm
addParameter(p,'StepSize',5);
% Default color blue
addParameter(p,'Color','b');
% Parse inputs
parse(p,radii,varargin{:});
axesHandle = p.Results.AxesHandle;
stepSize = p.Results.StepSize;
color = p.Results.Color;

if ~isempty(axesHandle)
    parent = axesHandle;
else
    % Create new figure
    figure;
    parent = gca;
end
% hold on
set(parent,'NextPlot','add');

xPos = 0;
yPos = 0;

% Plot cylinder for each radius (cylinder height = step size)
for i = 1:numel(radii)
    % Radius in mm
    r = radii(i);
    % Cylinder height = step size
    zPos = stepSize*(i-1);
    
    % Create circle points
    th = 0:pi/50:2*pi;
    xunit = r * cos(th) + xPos;
    yunit = r * sin(th) + yPos;
    zunit = repmat(zPos,size(xunit));
    
    % Plot bottom of cylinder
    bottom = fill3(zunit,xunit,yunit,color,'Parent',parent);
    % Plot top of cylinder
    top = fill3(zunit+stepSize,xunit,yunit,color,'Parent',parent);
    % Plot cylinder sides
    [x,y,z] = cylinder(r);
    surf(stepSize*z+zPos,x,y,'LineStyle','none','FaceColor',color,'Parent',parent);
end

% Set lighting and camera properties
if ~isempty(axesHandle)
    fig = findobj(get(0,'Children'),'Name','ProstateBrachyQA');
    set(fig,'CurrentAxes',axesHandle);
end
camlight;
lighting gouraud;
view(3);
axis equal;
% axis on;