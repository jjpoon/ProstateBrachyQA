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
        copiedAxes = copyobj(parent,fig, 'legacy');
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
            % Change legend text and background colour
            set(l,'TextColor','w','Color',[0.2 0.2 0.2]);
        end
end
end

