function showInFigure(parent,leg)
% SHOWINFIGURE opens the plotted image in a new figure window.
gui = ancestor(parent,'figure','toplevel');
switch get(gui,'SelectionType')
    case 'normal'
%         disp('single click')
    case 'open'
%         disp('double click')
        % Create new figure
        fig = figure('Units','normalized','Position',[0.1 0.1 0.8 0.8]);
        copiedAxes = copyobj(parent,fig);
        % Remove button down function for copied plot
        plots = get(copiedAxes,'Children');
        im = findobj(plots,'Type','image');
        set(im,'ButtonDownFcn','');
        % axesPos = get(parent,'Position');
        set(copiedAxes,'Units','normalized','Position',[0 0 1 1]);
        
        % Replot legend
        % Get original legend label
        labels = get(leg,'String');
        if ~isempty(labels)
            markers = zeros(1,numel(labels));
            for n = 1:numel(labels)
                labelStr = labels{n};
                % Get copied plot object with matching label name
                markers(n) = findobj(plots,'DisplayName',labelStr);
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
end

