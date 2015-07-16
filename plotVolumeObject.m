function plotVolumeObject(radii,varargin)

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

x = [];
y = [];
z = [];
% x = 0;
% y = 0;
% z = 0;

% Make contour for each radius
for i = 1:numel(radii)
    r = radii(i);
    % Z position of each contour is increased by step size
    zPos = stepSize*(i-1);
    % zPos = stepSize*i;
    
    % Get x,y,z coordinates of contour for current radius
    th = 0:pi/50:2*pi;
    xunit = r * cos(th) + xPos;
    yunit = r * sin(th) + yPos;
    zunit = repmat(zPos,size(xunit));
    
    % Plot contour line
    plot3(zunit,xunit,yunit,color,'Parent',parent);
%     fill3(xunit,yunit,zunit,color);
    
    % Store coordinates for all contours
    x = [x;xunit'];
    y = [y;yunit'];
    z = [z;zunit'];
end

points = [x,y,z];

% Interpolate closed spline passing through contours at x = 0
% Get coordinates of contour points where x = 0
curve = [];
for Z = 0:5:max(points(:,3))
    ind = find(round(points(:,1)*100)/100==0 & points(:,2)>=0 & round(points(:,3))==Z);
    if ~isempty(ind)
        curve = [curve; points(ind(1),:)];
    end
end
for Z = max(points(:,3)):-5:0
    ind = find(round(points(:,1)*100)/100==0 & points(:,2)<0 & round(points(:,3))==Z);
    if ~isempty(ind)
        curve = [curve; points(ind(1),:)];
    end
end
% Close the spline by doubling first point
curve = [curve; curve(1,:)];

% Interpolate the spline
spline = cscvn(curve');
splinePoints = fnplt(spline);

% plot3(splinePoints(1,:),splinePoints(2,:),splinePoints(3,:));

splineX = splinePoints(1,:);
splineY = splinePoints(2,:);
splineZ = splinePoints(3,:);

% Create 3D object from spline, if spline were rotated about z-axis
n=180;
t=(0:n)'*2*pi/n;
obj=surf(ones(n+1,1)*splineZ,cos(t)*splineY,sin(t)*splineY,...
    'EdgeColor','none','FaceColor',color,'Parent',parent);

% Set lighting and camera properties
if ~isempty(axesHandle)
    fig = findobj(get(0,'Children'),'Name','ProstateBrachyQA');
    set(fig,'CurrentAxes',axesHandle);
end
camlight;
lighting gouraud;
view(3);
axis equal;

end