function img = getAxesImage(varargin)
% GETAXESIMAGE copies the plots/legends displayed on the given axesHandle
% to a new figure, then uses getframe to save the image as a variable.

axesHandle = varargin{1};
figHandle = [];

if nargin > 1
    figHandle = varargin{2};
    if ~isempty(figHandle)
        % Figure has been created already, use this for plotting
        fig = figHandle;
        clf(fig);
        set(fig,'Units','normalized');
        set(fig,'Position',[0.1 0.1 0.8 0.8]);
    end
else
    % Create new figure
    fig = figure('Units','normalized','Position',[0.1 0.1 0.8 0.8],'Visible','off');
end
copiedAxes = copyobj(axesHandle,fig);
plots = get(copiedAxes,'Children');
set(plots,'Visible','on');
set(copiedAxes,'Units','normalized','Position',[0 0 1 1]);

% Get legend
leg = [];
im = findobj(plots,'Type','image');
imIndex = get(im,'UserData');
parentPanel = get(axesHandle,'Parent');
legends = findobj(get(parentPanel,'Children'),'Tag','legend');
for l = 1:numel(legends)
    userData = get(legends(l),'UserData');
    if userData.ImageIndex == imIndex
        leg = legends(l);
        break
    end
end

if ~isempty(leg)
    % Replot legend
    % Get original legend label
    labels = get(leg,'String');
    if ~isempty(labels)
        markers = zeros(1,numel(labels));
        for n = 1:numel(labels)
            labelStr = labels{n};
            % Get copied plot object with matching label name
            copiedPlot = findobj(plots,'DisplayName',labelStr);
            if numel(copiedPlot) > 1
                % If more than one plot with same label, take the one
                % that hasn't been copied yet
                ind = find(~ismember(copiedPlot,markers),1);
                markers(n) = copiedPlot(ind);
            else
                markers(n) = copiedPlot;
            end
        end
        % Create legend
        l = legend(markers,labels,'Location','southeast','Orientation','horizontal');
        % Decrease legend marker size
        markerObjs = findobj(get(l,'children'), 'type', 'line');
        set(markerObjs, 'Markersize', 12);
        % Change legend text and background colour
        set(l,'TextColor','w','Color',[0.2 0.2 0.2]);
    end
end

% print(fig,'AxesImage','-dpng','-r300');
% img = imread('AxesImage.png');
% delete('AxesImage.png');
frame = getframe(get(fig,'CurrentAxes'));
img = frame.cdata;

% If figure was given as input, don't delete
if isempty(figHandle)
    % Delete figure automatically if created inside this function
    delete(fig);
end

