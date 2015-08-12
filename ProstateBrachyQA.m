function varargout = ProstateBrachyQA(varargin)
% PROSTATEBRACHYQA MATLAB code for ProstateBrachyQA.fig
%      PROSTATEBRACHYQA, by itself, creates a new PROSTATEBRACHYQA or raises the existing
%      singleton*.
%
%      H = PROSTATEBRACHYQA returns the handle to a new PROSTATEBRACHYQA or the handle to
%      the existing singleton*.
%
%      PROSTATEBRACHYQA('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in PROSTATEBRACHYQA.M with the given input arguments.
%
%      PROSTATEBRACHYQA('Property','Value',...) creates a new PROSTATEBRACHYQA or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before ProstateBrachyQA_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to ProstateBrachyQA_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help ProstateBrachyQA

% Last Modified by GUIDE v2.5 12-Aug-2015 18:12:05

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @ProstateBrachyQA_OpeningFcn, ...
                   'gui_OutputFcn',  @ProstateBrachyQA_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before ProstateBrachyQA is made visible.
function ProstateBrachyQA_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to ProstateBrachyQA (see VARARGIN)

% Choose default command line output for ProstateBrachyQA
handles.output = hObject;

% Initialize scale readings
handles.upperScaleReading = cell(1,11);
handles.lowerScaleReading = cell(1,11);
% Volume formula test can have separate scale reading for axial/sagittal image
handles.upperScaleReading{10} = cell(1,2);
handles.lowerScaleReading{10} = cell(1,2);

% Set up tabs
% Create tab group
handles.tabgroup = uitabgroup(hObject,'Position',[0 0 1 1]);
% Create tabs
tab1 = uitab(handles.tabgroup,'Title','Phantom');
tab2 = uitab(handles.tabgroup,'Title','Grayscale');
tab3 = uitab(handles.tabgroup,'Title','Depth');
tab4 = uitab(handles.tabgroup,'Title','Axial Resolution');
tab5 = uitab(handles.tabgroup,'Title','Lateral Resolution');
tab6 = uitab(handles.tabgroup,'Title','Axial Distance');
tab7 = uitab(handles.tabgroup,'Title','Lateral Distance');
tab8 = uitab(handles.tabgroup,'Title','Area');
tab9 = uitab(handles.tabgroup,'Title','Volume (Planimetric)');
tab10 = uitab(handles.tabgroup,'Title','Volume (Formula)');
tab11 = uitab(handles.tabgroup,'Title','Grid Alignment');
tab12 = uitab(handles.tabgroup,'Title','Overlay Images');
% Set tabs as parents of appropriate test panels
set(handles.phantom_panel_parent,'Parent',tab1,'Position',[0 0 1 0.99]);
set(handles.grayscale_panel_parent,'Parent',tab2,'Position',[0 0 1 0.99]);
set(handles.depth_panel_parent,'Parent',tab3,'Position',[0 0 1 0.99]);
set(handles.axialResolution_panel_parent,'Parent',tab4,'Position',[0 0 1 0.99]);
set(handles.lateralResolution_panel_parent,'Parent',tab5,'Position',[0 0 1 0.99]);
set(handles.axialDistance_panel_parent,'Parent',tab6,'Position',[0 0 1 0.99]);
set(handles.lateralDistance_panel_parent,'Parent',tab7,'Position',[0 0 1 0.99]);
set(handles.area_panel_parent,'Parent',tab8,'Position',[0 0 1 0.99]);
set(handles.volume_panel_parent,'Parent',tab9,'Position',[0 0 1 0.99]);
set(handles.volumeFormula_panel_parent,'Parent',tab10,'Position',[0 0 1 0.99]);
set(handles.gridAlignment_panel_parent,'Parent',tab11,'Position',[0 0 1 0.99]);
set(handles.overlay_panel_parent,'Parent',tab12,'Position',[0 0 1 0.99]);

% Initialize images
handles.images = cell(11,1);

% Initialize testNum and testName
handles.testNum = 1;
handles.testName = 'grayscale';

% Initialize single view image index
handles.volume_imageIndex = 1;
handles.volumeFormula_imageIndex = 1;
handles.gridAlignment_imageIndex = 1;

% Initialize view plane for tests where axial/sagittal views are used
handles.depth_plane = 'axial';
handles.axialResolution_plane = 'axial';
handles.lateralResolution_plane = 'axial';
handles.lateralDistance_plane = 'axial';

% Initialize known values (if given, will use instead of value in baseline
% file)
handles.axialDistance_knownVal = {};
handles.lateralDistance_knownVal = {};
handles.area_knownVal = {};
handles.volume_knownVal = {};
handles.volumeFormula_knownVal = {};

% Initialize flags for scale warning
handles.AssumedScale = 0;
handles.ShowScaleWarning = 1;

% Initialize mouse movement fields
handles.rotate3D = 0;
handles.oldMousePoint = [];

% Initialize volume test 3DView option
handles.volume_3DView = 'interp';

% Initialize grid coordinates
handles.coordsPresetNum = [];
handles.gridAlignmentCoords = [];

% Initialize export images
handles.exportImages = cell(11,1);

% Initialize overlay images
handles.overlayImage1 = [];
handles.overlayImage2 = [];
handles.overlayImagePlot1 = [];
handles.overlayImagePlot2 = [];

% Initialize overlay image colour tint
handles.overlayImage1Colour = [1 1 1];
handles.overlayImage2Colour = [1 1 1];

% Add listener for selected tab index
set(handles.tabgroup,'SelectionChangedFcn',@(obj,eventdata)onSelectedTabChanged(hObject));

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes ProstateBrachyQA wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = ProstateBrachyQA_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

% --- Executes when user selects different tab
function onSelectedTabChanged(hObject)
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get updated handles
handles = guidata(hObject);

% Get test name (test function called and gui handles depend on this name)
tab = get(handles.tabgroup,'SelectedTab');
switch tab.Title
    case 'Phantom'
        testNum = 1;
        testName = 'phantom';
    case 'Grayscale'
        testNum = 2;
        testName = 'grayscale';
    case 'Depth'
        testNum = 3;
        testName = 'depth';
    case 'Axial Resolution'
        testNum = 4;
        testName = 'axialResolution';
    case 'Lateral Resolution'
        testNum = 5;
        testName = 'lateralResolution';
    case 'Axial Distance'
        testNum = 6;
        testName = 'axialDistance';
    case 'Lateral Distance'
        testNum = 7;
        testName = 'lateralDistance';
    case 'Area'
        testNum = 8;
        testName = 'area';
    case 'Volume (Planimetric)',
        testNum = 9;
        testName = 'volume';
    case 'Volume (Formula)'
        testNum = 10;
        testName = 'volumeFormula';
    case 'Grid Alignment'
        testNum = 11;
        testName = 'gridAlignment';
    case 'Overlay Images'
        testNum = 12;
        testName = 'overlay';
end
handles.testNum = testNum;
handles.testName = testName;
% Update handles
guidata(hObject,handles);


% --- Executes on Select Images button press.
function button_images_Callback(hObject, eventdata, handles)
% hObject    handle to grayscale_button_images (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get test number and name
testNum = handles.testNum;
testName = handles.testName;

filenames = 0;

% Open dialog for selecting image(s)
if strcmp(testName,'volumeFormula')
    [axialFile,axialPath] = uigetfile({'*.bmp;*.jpg;*.tif;*.png;*.gif;*.dcm','All Image Files';...
        '*.*','All Files' },'Select Axial Image');
    if axialPath ~= 0
        [sagittalFile,sagittalPath] = uigetfile({'*.bmp;*.jpg;*.tif;*.png;*.gif;*.dcm','All Image Files';...
            '*.*','All Files' },'Select Sagittal Image',axialPath);
        filenames = {axialFile, sagittalFile};
    end
else
    [filenames,pathname] = uigetfile({'*.bmp;*.jpg;*.tif;*.png;*.gif;*.dcm','All Image Files';...
        '*.*','All Files' },'Select Image(s)','MultiSelect','on');
end

if ischar(filenames) || isnumeric(filenames)
    filenames = {filenames};
end
% If all filenames are not 0 (0 if user pressed cancel)
if ~any(cellfun(@isnumeric,filenames))
    % Clear any old images for current test
    handles.images{testNum} = [];
    % Store new images
    if strcmp(testName,'volumeFormula')
        handles.images{testNum}{1} = imread(fullfile(axialPath,axialFile));
        handles.images{testNum}{2} = imread(fullfile(sagittalPath,sagittalFile));
    else
        for i = 1:numel(filenames)
            handles.images{testNum}{i} = imread(fullfile(pathname,filenames{i}));
        end
    end
    % Get the listbox that is also on this panel
    listbox = handles.([testName '_listbox']);
    % Set listbox 'Value' property
    set(listbox,'Value',numel(filenames));
    % Display filenames in listbox
    set(listbox,'String',filenames);
    % Enable Run Test button
    runTestButton = handles.([testName '_button_runTest']);
    set(runTestButton,'Enable','on');
    % Enable Flip Horizontal and Flip Vertical buttons
    set(handles.([testName '_button_flipHor']),'Enable','on');
    set(handles.([testName '_button_flipVert']),'Enable','on');
    
    % Show image preview
    if ~any(strcmp(testName,{'volume','volumeFormula','gridAlignment'}))
        % For all tests except volume and grid alignment
        % Clear axes
        axesHandle = handles.([testName '_axes']);
        cla(axesHandle);
        % Remove old legends
        parentPanel = get(axesHandle,'Parent');
        legends = findobj(get(parentPanel,'Children'),'Type','legend');
        delete(legends);
        
        im = imread(fullfile(pathname,filenames{1}));
        imPlot = imshow(im,'Parent',axesHandle);
        % Callback when double clicking on image
        set(imPlot,'ButtonDownFcn',@(obj,eventdata)showInFigure(axesHandle,[]));
    else
        % For volume and grid alignment tests, has grid and single views
        % Clear existing grid view
        panelHandle = handles.([testName '_panel_figure']);
        delete(get(panelHandle,'Children'));
        % Clear axes
        axesHandle = handles.([testName '_axes']);
        cla(axesHandle);
        % Delete any existing old axes (except original)
        if isfield(handles,[testName '_axes_list'])
            delete(handles.([testName '_axes_list'])(2:end));
        end
        % Remove old legends
        parentPanel = get(axesHandle,'Parent');
        legends = findobj(get(parentPanel,'Children'),'Type','legend');
        delete(legends);
        
        % Create separate axes for each image
        axesHandles = zeros(numel(handles.images{testNum}),1);
        axesHandles(1) = axesHandle;
        for n = 2:numel(handles.images{testNum})
            % Create copies of existing testName_axes
            axesHandles(n) = copyobj(axesHandle,handles.([testName '_panel']), 'legacy');
        end
        % Store axes handles list
        handles.([testName '_axes_list']) = axesHandles;
        % Bring panel_figure back on top
        uistack(panelHandle,'top');
        
        % Reset image index
        handles.([testName '_imageIndex']) = 1;
        
        % Set size of image grid based on number of inputted images
        numImages = numel(filenames);
        gridSize = [round(sqrt(numImages)),ceil(sqrt(numImages))];
        n = gridSize(1);
        m = gridSize(2);
        
        % Plot images
        for i = 1:numImages
            if strcmp(testName,'volumeFormula')
                if i == 1
                    pathname = axialPath;
                else
                    pathname = sagittalPath;
                end
            end
            im = imread(fullfile(pathname,filenames{i}));
            % Create subplots
            [c,r] = ind2sub([gridSize(2) gridSize(1)], i);
            parent = subplot('Position', [(c-1)/m, 1-(r)/n, 1/m, 1/n],'Parent',panelHandle);
            % Plot grid view image
            imPlot_grid = imshow(im,'Parent',parent);
            % Store image index
            set(imPlot_grid,'UserData',i);
            % Callback when double clicking on image
            set(imPlot_grid,'ButtonDownFcn',@(obj,eventdata)showInFigure(parent,[]));
            
            % Plot single view image
            imPlot_single = imshow(im,'Parent',axesHandles(i));
            % Only show first image, hide others
            if i>1
                set(imPlot_single,'Visible','off');
            end
            % Store image index
            set(imPlot_single,'UserData',i);
            % Callback when double clicking on image
            set(imPlot_single,'ButtonDownFcn',@(obj,eventdata)showInFigure(axesHandles(i),[]));
        end
        
        % If in single view mode, show prev and next buttons
        if get(handles.([testName '_button_singleView']),'Value') == 1
            set(handles.([testName '_button_prev']),'Visible','on');
            set(handles.([testName '_button_next']),'Visible','on');
        end
        
    end
    
    % Enable button to assign grid coordinates to each image
    if strcmp(testName,'gridAlignment')
        handles.gridAlignmentCoords = repmat({''},size(filenames));
        set(handles.gridAlignment_button_assignCoords,'Enable','on');
    end
    
end
guidata(hObject,handles);


% --- Executes during object creation, after setting all properties.
function grayscale_listbox_CreateFcn(hObject, eventdata, handles)
% hObject    handle to grayscale_listbox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in grayscale_button_setScale.
function button_setScale_Callback(hObject, eventdata, handles)
% hObject    handle to grayscale_button_setScale (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get test number
testNum = handles.testNum;

% Manually input upper and lower scale readings
prompt = {'Enter the upper scale reading (cm):',...
    'Enter the lower scale reading (cm):'};
title = 'Manually input scale readings (optional)';
numlines = [1, length(title)+10];
default = {num2str(handles.upperScaleReading{testNum}),num2str(handles.lowerScaleReading{testNum})};
answer=inputdlg(prompt,title,numlines,default);
% If user inputted both numbers
if ~isempty(answer)
    handles.upperScaleReading{testNum} = str2num(answer{1});
    handles.lowerScaleReading{testNum} = str2num(answer{2});
end
guidata(hObject,handles);


% --- Executes on button press in volume_button_gridView.
function button_gridView_Callback(hObject, eventdata, handles)
% hObject    handle to volume_button_gridView (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of volume_button_gridView
state = get(hObject,'Value');
% Toggle other button
testName = handles.testName;
set(handles.([testName '_button_singleView']),'Value',~state);
% Disable button when toggled on
if state == 1
    set(hObject,'Enable','off')
    set(handles.([testName '_button_singleView']),'Enable','on')
end

% Hide all single view axes plots
if isfield(handles,[testName '_axes_list'])
    plots = findall(handles.([testName '_axes_list']));
    set(plots,'Visible','off');
    % Hide all associated legends
    testPanelChildren = get(handles.([testName '_panel']),'Children');
    legends = findobj(testPanelChildren,'flat','Type','legend');
    set(legends,'Visible','off');
end
% Show grid view panel
set(handles.([testName '_panel_figure']),'Visible','on');

% Hide previous and next image buttons
set(handles.([testName '_button_prev']),'Visible','off');
set(handles.([testName '_button_next']),'Visible','off');

guidata(hObject,handles);


% --- Executes on button press in volume_button_singleView.
function button_singleView_Callback(hObject, eventdata, handles)
% hObject    handle to volume_button_singleView (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of volume_button_singleView
state = get(hObject,'Value');
% Toggle other button
testName = handles.testName;
set(handles.([testName '_button_gridView']),'Value',~state);
% Disable button when toggled on
if state == 1
    set(hObject,'Enable','off')
    set(handles.([testName '_button_gridView']),'Enable','on')
end

% Get image index
imageIndex = handles.([testName '_imageIndex']);

% Hide grid view axes plots
set(handles.([testName '_panel_figure']),'Visible','off');
% Show single view axes for current image
if isfield(handles,[testName '_axes_list'])
    currImageAxes = handles.([testName '_axes_list'])(imageIndex);
    plots = get(currImageAxes,'Children');
    set(plots,'Visible','on');
    % Show legend associated with current image
    testPanelChildren = get(handles.([testName '_panel']),'Children');
    legends = findobj(testPanelChildren,'Type','legend');
    for n = 1:numel(legends)
        leg = legends(n);
        userData = get(leg,'UserData');
        if userData.ImageIndex == imageIndex
            set(leg,'Visible','on');
        end
    end
    
    % Show previous and next image buttons
    set(handles.([testName '_button_prev']),'Visible','on');
    set(handles.([testName '_button_next']),'Visible','on');
end

% Update handles
guidata(hObject,handles);


% --- Executes on button press in volume_button_prev.
function button_prev_Callback(hObject, eventdata, handles)
% hObject    handle to volume_button_prev (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Update image index
testNum = handles.testNum;
testName = handles.testName;
imageIndex = handles.([testName '_imageIndex']);
if imageIndex > 1
    % Decrease image index by 1
    imageIndex = imageIndex - 1;
else
    % If on first image, switch index to last image
    imageIndex = numel(handles.images{testNum});
end
handles.([testName '_imageIndex']) = imageIndex;

% Hide currently shown axes
if isfield(handles,[testName '_axes_list'])
    plots = findall(handles.([testName '_axes_list']));
    set(plots,'Visible','off');
    % Hide all associated legends
    testPanelChildren = get(handles.([testName '_panel']),'Children');
    legends = findobj(testPanelChildren,'flat','Type','legend');
    set(legends,'Visible','off');
end
% Show previous image plots
ax = handles.([testName '_axes_list'])(imageIndex);
plots = get(ax,'Children');
set(plots,'Visible','on');
% Show legend associated with previous image
testPanelChildren = get(handles.([testName '_panel']),'Children');
legends = findobj(testPanelChildren,'Type','legend');
for n = 1:numel(legends)
    leg = legends(n);
    userData = get(leg,'UserData');
    if userData.ImageIndex == imageIndex
        set(leg,'Visible','on');
    end
end

% Update handles
guidata(hObject,handles);

% --- Executes on button press in volume_button_next.
function button_next_Callback(hObject, eventdata, handles)
% hObject    handle to volume_button_next (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Update image index
testNum = handles.testNum;
testName = handles.testName;
imageIndex = handles.([testName '_imageIndex']);
if imageIndex < numel(handles.images{testNum})
    % Increase image index by 1
    imageIndex = imageIndex + 1;
else
    % If on last image, reset index to first image
    imageIndex = 1;
end
handles.([testName '_imageIndex']) = imageIndex;

% Hide currently shown axes
if isfield(handles,[testName '_axes_list'])
    plots = findall(handles.([testName '_axes_list']));
    set(plots,'Visible','off');
    % Hide all associated legends
    testPanelChildren = get(handles.([testName '_panel']),'Children');
    legends = findobj(testPanelChildren,'flat','Type','legend');
    set(legends,'Visible','off');
end
% Show next image axes
ax = handles.([testName '_axes_list'])(imageIndex);
plots = get(ax,'Children');
set(plots,'Visible','on');
% Show legend associated with next image
testPanelChildren = get(handles.([testName '_panel']),'Children');
legends = findobj(testPanelChildren,'Type','legend');
for n = 1:numel(legends)
    leg = legends(n);
    userData = get(leg,'UserData');
    if userData.ImageIndex == imageIndex
        set(leg,'Visible','on');
    end
end

% Update handles
guidata(hObject,handles);


% --- Executes when figure1 is resized.
function figure1_ResizeFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Set all units to normalized
set(findobj(hObject,'-property','Units'),'Units','normalized');

for str = {'volume','gridAlignment'}
    testName = str{1};
    % Get previous button position
    prevPos = get(handles.([testName '_button_prev']),'Position');
    % Get axes position
    axesPos = get(handles.([testName '_axes']),'Position');
    % Set new button positions to bottom center of axes
    newPrevPos = [0.5-prevPos(3), axesPos(2)+0.01, prevPos(3), prevPos(4)];
    newNextPos = [0.5, axesPos(2)+0.01, prevPos(3), prevPos(4)];
    set(handles.([testName '_button_prev']),'Position',newPrevPos);
    set(handles.([testName '_button_next']),'Position',newNextPos);
end


% --- Executes during object creation, after setting all properties.
function grayscale_table_CreateFcn(hObject, eventdata, handles)
% hObject    handle to grayscale_table (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

data = cell(1,3);
set(hObject,'Data',data);
set(hObject,'RowName',{'Gradient Length'});
set(hObject,'ColumnName',{'Baseline (mm)','Current (mm)','Result'});
set(hObject,'ColumnEditable',false(size(data)));


% --- Executes on button press in grayscale_button_runTest.
function grayscale_button_runTest_Callback(hObject, eventdata, handles)
% hObject    handle to grayscale_button_runTest (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Show loading cursor
oldpointer = get(handles.figure1,'pointer');
set(handles.figure1,'pointer','watch') 
drawnow;

try
    % Get test number and name (test function called and gui handles depend on this name)
    testNum = handles.testNum;
    axesHandle = handles.grayscale_axes;
    
    if numel(handles.images) >= testNum
        if ~isempty(handles.images{testNum})
            
            % Clear axes
            cla(axesHandle);
            % Remove old legends
            parentPanel = get(axesHandle,'Parent');
            legends = findobj(get(parentPanel,'Children'),'Type','legend');
            delete(legends);
            
            % Run test, plot on given axes
            % Check if scale readings were set manually
            if ~isempty(handles.upperScaleReading{testNum}) && ~isempty(handles.lowerScaleReading{testNum})
                % Scale readings were inputted
                [result,baselineVal,newVal] = grayscaleTestAuto(handles.images{testNum}{:},...
                    'UpperScale',handles.upperScaleReading{testNum},'LowerScale',handles.lowerScaleReading{testNum},...
                    'AxesHandle',axesHandle);
            else
                % Read scale automatically from image
                [result,baselineVal,newVal] = grayscaleTestAuto(handles.images{testNum}{:},'AxesHandle',axesHandle);
            end
            
            % Get updated handles
            handles = guidata(hObject);
            
            % Display values with 2 decimal places
            baselineValText = sprintf('%.2f',baselineVal);
            newValText = sprintf('%.2f',newVal);
            
            % Modify table data
            table = handles.grayscale_table;
            data = get(table,'Data');
            % Set baseline value table cell
            data{1,1} = baselineValText;
            % Set new value table cell
            data{1,2} = newValText;
            % Set test result table cell
            if result == 1
                data{1,3} = '<html><font color="green">PASS';
            else
                data{1,3} = '<html><font color="red">FAIL';
            end
            % Set table data
            set(table,'Data',data);
            
            % Enable Set Baseline button
            set(handles.grayscale_button_setBaseline,'Enable','on');
            
            % Show scale warning if needed
            showScaleWarning(hObject,handles);
            % Get updated handles
            handles = guidata(hObject);
            
            % Store image for export
            im = getframe(handles.grayscale_axes);
            handles.exportImages{testNum}{1} = im.cdata;
        end
    end
catch exception
    disp(getReport(exception));
end

% Set cursor back to normal
set(handles.figure1,'pointer',oldpointer);

guidata(hObject,handles);


% --- Executes during object creation, after setting all properties.
function depth_table_CreateFcn(hObject, eventdata, handles)
% hObject    handle to depth_table_axial (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

data = cell(1,3);
set(hObject,'Data',data);
set(hObject,'RowName',{'Depth'});
set(hObject,'ColumnName',{'Baseline (mm)','Current (mm)','Result'});
set(hObject,'ColumnEditable',false(size(data)));


% --- Executes on button press in depth_button_runTest.
function depth_button_runTest_Callback(hObject, eventdata, handles)
% hObject    handle to depth_button_runTest (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Show loading cursor
oldpointer = get(handles.figure1,'pointer');
set(handles.figure1,'pointer','watch') 
drawnow;

try
    % Get test number and name (test function called and gui handles depend on this name)
    testNum = handles.testNum;
    axesHandle = handles.depth_axes;
    
    if numel(handles.images) >= testNum
        if ~isempty(handles.images{testNum})
            
            % Clear axes
            cla(axesHandle);
            % Remove old legends
            parentPanel = get(axesHandle,'Parent');
            legends = findobj(get(parentPanel,'Children'),'Type','legend');
            delete(legends);
            
            % Run test, plot on given axes
            % Check if scale readings were set manually
            if ~isempty(handles.upperScaleReading{testNum}) && ~isempty(handles.lowerScaleReading{testNum})
                % Scale readings were inputted
                [result,baselineVal,newVal,freq] = depthTestAuto(handles.images{testNum}{:},...
                    'UpperScale',handles.upperScaleReading{testNum},'LowerScale',handles.lowerScaleReading{testNum},...
                    'AxesHandle',axesHandle,'Plane',handles.depth_plane);
            else
                % Read scale automatically from image
                [result,baselineVal,newVal,freq] = depthTestAuto(handles.images{testNum}{:},...
                    'AxesHandle',axesHandle,'Plane',handles.depth_plane);
            end
            
            % Get updated handles
            handles = guidata(hObject);
            
            % Display values with 2 decimal places
            baselineValText = sprintf('%.2f',baselineVal);
            newValText = sprintf('%.2f',newVal);
            
            % Modify results data
            if strcmp(handles.depth_plane,'axial')
                table = handles.depth_table_axial;
                freqText = handles.depth_text_freqAxial;
            elseif strcmp(handles.depth_plane,'longitudinal')
                table = handles.depth_table_long;
                freqText = handles.depth_text_freqLong;
            end
            
            data = get(table,'Data');
            % Set baseline value table cell
            data{1,1} = baselineValText;
            % Set new value table cell
            data{1,2} = newValText;
            % Set test result table cell
            if isempty(result)
                data{1,3} = [];
            elseif result == 1
                data{1,3} = '<html><font color="green">PASS';
            else
                data{1,3} = '<html><font color="red">FAIL';
            end
            % Set table data
            set(table,'Data',data);
            
            % Show frequency
            set(freqText,'String',num2str(freq));
            
            % Enable Set Baseline button
            set(handles.depth_button_setBaseline,'Enable','on');
            
            % Show scale warning if needed
            showScaleWarning(hObject,handles);
            % Get updated handles
            handles = guidata(hObject);
            
            % Store image for export
            im = getframe(handles.depth_axes);
            if strcmp(handles.depth_plane,'axial')
                handles.exportImages{testNum}{1} = im.cdata;
            elseif strcmp(handles.depth_plane,'longitudinal')
                handles.exportImages{testNum}{2} = im.cdata;
            end
        end
    end
catch exception
    disp(getReport(exception));
end

% Set cursor back to normal
set(handles.figure1,'pointer',oldpointer);

guidata(hObject,handles);


% --- Executes when selected object is changed in depth_buttongroup_result.
function buttongroup_result_SelectionChangedFcn(hObject, eventdata, handles)
% hObject    handle to the selected object in depth_buttongroup_result 
% eventdata  structure with the following fields (see UIBUTTONGROUP)
%	EventName: string 'SelectionChanged' (read only)
%	OldValue: handle of the previously selected object or empty if none was selected
%	NewValue: handle of the currently selected object
% handles    structure with handles and user data (see GUIDATA)

% Get updated handles
% handles = guidata(hObject);
testName = handles.testName;

selection = get(eventdata.NewValue,'Tag');
if ~isempty(strfind(selection,'radiobutton_axial'))
    % Axial plane
    handles.([testName '_plane']) = 'axial';
else
    % Longitudinal plane
    handles.([testName '_plane']) = 'longitudinal';
end
% Update handles
guidata(hObject,handles);


% --- Executes during object creation, after setting all properties.
function axialResolution_table_axial_CreateFcn(hObject, eventdata, handles)
% hObject    handle to axialResolution_table_axial (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

data = cell(4,5);
set(hObject,'Data',data);
set(hObject,'RowName',{'Proximal (B1)','Proximal (F1)','Distal (B5)','Distal (F5)'});
set(hObject,'ColumnName',{'Baseline (mm)','Current (mm)','Diff (abs)','Diff (%)','Result'});
set(hObject,'ColumnEditable',false(1,size(data,2)));

% --- Executes during object creation, after setting all properties.
function axialResolution_table_long_CreateFcn(hObject, eventdata, handles)
% hObject    handle to axialResolution_table_axial (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

data = cell(2,5);
set(hObject,'Data',data);
set(hObject,'RowName',{'Filament 1','Filament 6'});
set(hObject,'ColumnName',{'Baseline (mm)','Current (mm)','Diff (abs)','Diff (%)','Result'});
set(hObject,'ColumnEditable',false(1,size(data,2)));

% --- Executes on button press in axialResolution_button_runTest.
function axialResolution_button_runTest_Callback(hObject, eventdata, handles)
% hObject    handle to axialResolution_button_runTest (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Show loading cursor
oldpointer = get(handles.figure1,'pointer');
set(handles.figure1,'pointer','watch') 
drawnow;

try
    % Get test number and name (test function called and gui handles depend on this name)
    testNum = handles.testNum;
    axesHandle = handles.axialResolution_axes;
    
    if numel(handles.images) >= testNum
        if ~isempty(handles.images{testNum})
            
            % Clear axes
            cla(axesHandle);
            % Remove old legends
            parentPanel = get(axesHandle,'Parent');
            legends = findobj(get(parentPanel,'Children'),'Type','legend');
            delete(legends);
            
            % Run test, plot on given axes
            % Check if scale readings were set manually
            if ~isempty(handles.upperScaleReading{testNum}) && ~isempty(handles.lowerScaleReading{testNum})
                % Scale readings were inputted
                [result,baselineVals,newVals,freq] = axialResolutionTestAuto(handles.images{testNum}{:},...
                    'UpperScale',handles.upperScaleReading{testNum},'LowerScale',handles.lowerScaleReading{testNum},...
                    'AxesHandle',axesHandle);
            else
                % Read scale automatically from image
                [result,baselineVals,newVals,freq] = axialResolutionTestAuto(handles.images{testNum}{:},'AxesHandle',axesHandle);
            end
            
            % Get updated handles
            handles = guidata(hObject);
            
            % Get results data
            if strcmp(handles.axialResolution_plane,'axial')
                baselines = baselineVals(1:4);
                table = handles.axialResolution_table_axial;
                freqText = handles.axialResolution_text_freqAxial;
            elseif strcmp(handles.axialResolution_plane,'longitudinal')
                baselines = baselineVals(5:6);
                table = handles.axialResolution_table_long;
                freqText = handles.axialResolution_text_freqLong;
            end
            
            % Modify table data
            data = get(table,'Data');
            for n = 1:numel(newVals)
                data{n,1} = sprintf('%.2f',baselines(n));
                data{n,2} = sprintf('%.2f',newVals(n));
                % Absolute difference
                absDiff = abs(newVals(n)-baselines(n));
                data{n,3} = sprintf('%.2f',absDiff);
                % Percent difference
                avg = (baselines(n)+newVals(n))/2;
                percentDiff = absDiff/avg*100;
                data{n,4} = sprintf('%.2f',percentDiff);
                % Result
                if isempty(result)
                    data{n,5} = [];
                elseif result(n) == 1
                    data{n,5} = '<html><font color="green">PASS';
                else
                    data{n,5} = '<html><font color="red">FAIL';
                end
            end
            % Set table data
            set(table,'Data',data);
            
            % Show frequency
            set(freqText,'String',num2str(freq));
            
            % Enable Set Baseline button
            set(handles.axialResolution_button_setBaseline,'Enable','on')
            
            % Show scale warning if needed
            showScaleWarning(hObject,handles);
            % Get updated handles
            handles = guidata(hObject);
            
            % Store image for export
            im = getframe(handles.axialResolution_axes);
            if strcmp(handles.axialResolution_plane,'axial')
                handles.exportImages{testNum}{1} = im.cdata;
            elseif strcmp(handles.axialResolution_plane,'longitudinal')
                handles.exportImages{testNum}{2} = im.cdata;
            end
        end
    end
catch exception
    disp(getReport(exception));
end

% Set cursor back to normal
set(handles.figure1,'pointer',oldpointer);

guidata(hObject,handles);


% --- Executes during object creation, after setting all properties.
function lateralResolution_table_axial_CreateFcn(hObject, eventdata, handles)
% hObject    handle to axialResolution_table_axial (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

data = cell(4,5);
set(hObject,'Data',data);
set(hObject,'RowName',{'Proximal (B1)','Proximal (F1)','Distal (B5)','Distal (F5)'});
set(hObject,'ColumnName',{'Baseline (mm)','Current (mm)','Diff (abs)','Diff (%)','Result'});
set(hObject,'ColumnEditable',false(1,size(data,2)));

% --- Executes during object creation, after setting all properties.
function lateralResolution_table_long_CreateFcn(hObject, eventdata, handles)
% hObject    handle to axialResolution_table_axial (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

data = cell(2,5);
set(hObject,'Data',data);
set(hObject,'RowName',{'Filament 1','Filament 6'});
set(hObject,'ColumnName',{'Baseline (mm)','Current (mm)','Diff (abs)','Diff (%)','Result'});
set(hObject,'ColumnEditable',false(1,size(data,2)));

% --- Executes on button press in axialResolution_button_runTest.
function lateralResolution_button_runTest_Callback(hObject, eventdata, handles)
% hObject    handle to axialResolution_button_runTest (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Show loading cursor
oldpointer = get(handles.figure1,'pointer');
set(handles.figure1,'pointer','watch') 
drawnow;

try
    % Get test number and name (test function called and gui handles depend on this name)
    testNum = handles.testNum;
    axesHandle = handles.lateralResolution_axes;
    
    if numel(handles.images) >= testNum
        if ~isempty(handles.images{testNum})
            
            % Clear axes
            cla(axesHandle);
            % Remove old legends
            parentPanel = get(axesHandle,'Parent');
            legends = findobj(get(parentPanel,'Children'),'Type','legend');
            delete(legends);
            
            % Run test, plot on given axes
            % Check if scale readings were set manually
            if ~isempty(handles.upperScaleReading{testNum}) && ~isempty(handles.lowerScaleReading{testNum})
                % Scale readings were inputted
                [result,baselineVals,newVals] = lateralResolutionTestAuto(handles.images{testNum}{:},...
                    'UpperScale',handles.upperScaleReading{testNum},'LowerScale',handles.lowerScaleReading{testNum},...
                    'AxesHandle',axesHandle);
            else
                % Read scale automatically from image
                [result,baselineVals,newVals] = lateralResolutionTestAuto(handles.images{testNum}{:},'AxesHandle',axesHandle);
            end
            
            % Get updated handles
            handles = guidata(hObject);
            
            % Get table data
            if strcmp(handles.lateralResolution_plane,'axial')
                baselines = baselineVals(1:4);
                table = handles.lateralResolution_table_axial;
            elseif strcmp(handles.lateralResolution_plane,'longitudinal')
                baselines = baselineVals(5:6);
                table = handles.lateralResolution_table_long;
            end
            data = get(table,'Data');
            % Modify table data
            for n = 1:numel(newVals)
                data{n,1} = sprintf('%.2f',baselines(n));
                data{n,2} = sprintf('%.2f',newVals(n));
                % Absolute difference
                absDiff = abs(newVals(n)-baselines(n));
                data{n,3} = sprintf('%.2f',absDiff);
                % Percent difference
                avg = (baselines(n)+newVals(n))/2;
                percentDiff = absDiff/avg*100;
                data{n,4} = sprintf('%.2f',percentDiff);
                % Result
                if result(n) == 1
                    data{n,5} = '<html><font color="green">PASS';
                else
                    data{n,5} = '<html><font color="red">FAIL';
                end
            end
            % Set table data
            set(table,'Data',data);
            
            % Enable Set Baseline button
            set(handles.lateralResolution_button_setBaseline,'Enable','on')
            
            % Show scale warning if needed
            showScaleWarning(hObject,handles);
            % Get updated handles
            handles = guidata(hObject);
            
            % Store image for export
            im = getframe(handles.lateralResolution_axes);
            if strcmp(handles.lateralResolution_plane,'axial')
                handles.exportImages{testNum}{1} = im.cdata;
            elseif strcmp(handles.lateralResolution_plane,'longitudinal')
                handles.exportImages{testNum}{2} = im.cdata;
            end
        end
    end
catch exception
    disp(getReport(exception));
end

% Set cursor back to normal
set(handles.figure1,'pointer',oldpointer);

guidata(hObject,handles);


% --- Executes during object creation, after setting all properties.
function axialDistance_table_CreateFcn(hObject, eventdata, handles)
% hObject    handle to axialDistance_table (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

data = cell(2,5);
set(hObject,'Data',data);
set(hObject,'RowName',{'B1 - B5','F1 - F5'});
set(hObject,'ColumnName',{'Known (mm)','Measured (mm)','Diff (abs)','Diff (%)','Result'});
set(hObject,'ColumnEditable',false(1,size(data,2)));


% --- Executes on button press in axialDistance_button_runTest.
function axialDistance_button_runTest_Callback(hObject, eventdata, handles)
% hObject    handle to axialDistance_button_runTest (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Show loading cursor
oldpointer = get(handles.figure1,'pointer');
set(handles.figure1,'pointer','watch') 
drawnow;

try
    % Get test number and name (test function called and gui handles depend on this name)
    testNum = handles.testNum;
    axesHandle = handles.axialDistance_axes;
    
    if numel(handles.images) >= testNum
        if ~isempty(handles.images{testNum})
            
            % Clear axes
            cla(axesHandle);
            % Remove old legends
            parentPanel = get(axesHandle,'Parent');
            legends = findobj(get(parentPanel,'Children'),'Type','legend');
            delete(legends);
            
            % Run test, plot on given axes
            % Check if scale readings were set manually
            if ~isempty(handles.upperScaleReading{testNum}) && ~isempty(handles.lowerScaleReading{testNum})
                % Scale readings were inputted
                [result,knownVal,measuredVals] = axialDistanceTestAuto(handles.images{testNum}{:},...
                    'UpperScale',handles.upperScaleReading{testNum},'LowerScale',handles.lowerScaleReading{testNum},...
                    'AxesHandle',axesHandle);
            else
                % Read scale automatically from image
                [result,knownVal,measuredVals] = axialDistanceTestAuto(handles.images{testNum}{:},'AxesHandle',axesHandle);
            end
            
            % Get updated handles
            handles = guidata(hObject);
            
            % Get table data
            table = handles.axialDistance_table;
            data = get(table,'Data');
            % Modify table data
            for n = 1:numel(measuredVals)
                data{n,1} = sprintf('%.2f',knownVal);
                data{n,2} = sprintf('%.2f',measuredVals(n));
                % Absolute difference
                absDiff = abs(measuredVals(n)-knownVal);
                data{n,3} = sprintf('%.2f',absDiff);
                % Percent difference
                avg = (knownVal+measuredVals(n))/2;
                percentDiff = absDiff/avg*100;
                data{n,4} = sprintf('%.2f',percentDiff);
                % Result
                if ~isempty(result)
                    if result(n) == 1
                        data{n,5} = '<html><font color="green">PASS';
                    else
                        data{n,5} = '<html><font color="red">FAIL';
                    end
                end
            end
            % Set table data
            set(table,'Data',data);
            
            % Show scale warning if needed
            showScaleWarning(hObject,handles);
            % Get updated handles
            handles = guidata(hObject);
            
            % Store image for export
            im = getframe(handles.axialDistance_axes);
            handles.exportImages{testNum}{1} = im.cdata;
        end
    end
catch exception
    disp(getReport(exception));
end

% Set cursor back to normal
set(handles.figure1,'pointer',oldpointer);

guidata(hObject,handles);


% --- Executes during object creation, after setting all properties.
function lateralDistance_table_axial_CreateFcn(hObject, eventdata, handles)
% hObject    handle to lateralDistance_table_axial (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
data = cell(2,5);
set(hObject,'Data',data);
set(hObject,'RowName',{'Proximal','Distal'});
set(hObject,'ColumnName',{'Known (mm)','Measured (mm)','Diff (abs)','Diff (%)','Result'});
set(hObject,'ColumnEditable',false(1,size(data,2)));


% --- Executes during object creation, after setting all properties.
function lateralDistance_table_long_CreateFcn(hObject, eventdata, handles)
% hObject    handle to lateralDistance_table_long (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
data = cell(1,5);
set(hObject,'Data',data);
set(hObject,'RowName',{'Distance'});
set(hObject,'ColumnName',{'Known (mm)','Measured (mm)','Diff (abs)','Diff (%)','Result'});
set(hObject,'ColumnEditable',false(1,size(data,2)));


% --- Executes on button press in lateralDistance_button_runTest.
function lateralDistance_button_runTest_Callback(hObject, eventdata, handles)
% hObject    handle to lateralDistance_button_runTest (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Show loading cursor
oldpointer = get(handles.figure1,'pointer');
set(handles.figure1,'pointer','watch') 
drawnow;

try
    % Get test number and name (test function called and gui handles depend on this name)
    testNum = handles.testNum;
    axesHandle = handles.lateralDistance_axes;
    
    if numel(handles.images) >= testNum
        if ~isempty(handles.images{testNum})
            
            % Clear axes
            cla(axesHandle);
            % Remove old legends
            parentPanel = get(axesHandle,'Parent');
            legends = findobj(get(parentPanel,'Children'),'Type','legend');
            delete(legends);
            
            % Run test, plot on given axes
            % Check if scale readings were set manually
            if ~isempty(handles.upperScaleReading{testNum}) && ~isempty(handles.lowerScaleReading{testNum})
                % Scale readings were inputted
                [result,knownVal,measuredVals] = lateralDistanceTestAuto(handles.images{testNum}{:},...
                    'UpperScale',handles.upperScaleReading{testNum},'LowerScale',handles.lowerScaleReading{testNum},...
                    'AxesHandle',axesHandle);
            else
                % Read scale automatically from image
                [result,knownVal,measuredVals] = lateralDistanceTestAuto(handles.images{testNum}{:},'AxesHandle',axesHandle);
            end
            
            % Get updated handles
            handles = guidata(hObject);
            
            % Get table data
            if strcmp(handles.lateralDistance_plane,'axial')
                table = handles.lateralDistance_table_axial;
            elseif strcmp(handles.lateralDistance_plane,'longitudinal')
                table = handles.lateralDistance_table_long;
            end
            data = get(table,'Data');
            % Modify table data
            for n = 1:numel(measuredVals)
                data{n,1} = sprintf('%.2f',knownVal);
                data{n,2} = sprintf('%.2f',measuredVals(n));
                % Absolute difference
                absDiff = abs(measuredVals(n)-knownVal);
                data{n,3} = sprintf('%.2f',absDiff);
                % Percent difference
                avg = (knownVal+measuredVals(n))/2;
                percentDiff = absDiff/avg*100;
                data{n,4} = sprintf('%.2f',percentDiff);
                % Result
                if ~isempty(result)
                    if result(n) == 1
                        data{n,5} = '<html><font color="green">PASS';
                    else
                        data{n,5} = '<html><font color="red">FAIL';
                    end
                end
            end
            % Set table data
            set(table,'Data',data);
            
            % Show scale warning if needed
            showScaleWarning(hObject,handles);
            % Get updated handles
            handles = guidata(hObject);
            
            % Store image for export
            im = getframe(handles.lateralDistance_axes);
            if strcmp(handles.lateralDistance_plane,'axial')
                handles.exportImages{testNum}{1} = im.cdata;
            elseif strcmp(handles.lateralDistance_plane,'longitudinal')
                handles.exportImages{testNum}{2} = im.cdata;
            end
        end
    end
catch exception
    disp(getReport(exception));
end

% Set cursor back to normal
set(handles.figure1,'pointer',oldpointer);

guidata(hObject,handles);


% --- Executes during object creation, after setting all properties.
function area_table_CreateFcn(hObject, eventdata, handles)
% hObject    handle to area_table (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
data = cell(1,5);
set(hObject,'Data',data);
set(hObject,'RowName',{'Area'});
set(hObject,'ColumnName',{'<html>Known (cm<sup>2</sup>)</html>','<html>Measured (cm<sup>2</sup>)</html>',...
    'Diff (abs)','Diff (%)','Result'});
set(hObject,'ColumnEditable',false(1,size(data,2)));

% --- Executes on button press in area_button_runTest.
function area_button_runTest_Callback(hObject, eventdata, handles)
% hObject    handle to area_button_runTest (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Get test number and name (test function called and gui handles depend on this name)

% Show loading cursor
oldpointer = get(handles.figure1,'pointer');
set(handles.figure1,'pointer','watch') 
drawnow;

try
    testNum = handles.testNum;
    axesHandle = handles.area_axes;
    
    if numel(handles.images) >= testNum
        if ~isempty(handles.images{testNum})
            
            % Clear axes
            cla(axesHandle);
            % Remove old legends
            parentPanel = get(axesHandle,'Parent');
            legends = findobj(get(parentPanel,'Children'),'Type','legend');
            delete(legends);
            
            % Run test, plot on given axes
            % Check if scale readings were set manually
            if ~isempty(handles.upperScaleReading{testNum}) && ~isempty(handles.lowerScaleReading{testNum})
                % Scale readings were inputted
                [result,knownVal,measuredVal] = areaTestAuto(handles.images{testNum}{:},...
                    'UpperScale',handles.upperScaleReading{testNum},'LowerScale',handles.lowerScaleReading{testNum},...
                    'AxesHandle',axesHandle);
            else
                % Read scale automatically from image
                [result,knownVal,measuredVal] = areaTestAuto(handles.images{testNum}{:},'AxesHandle',axesHandle);
            end
            
            % Get updated handles
            handles = guidata(hObject);
            
            % Get table data
            table = handles.area_table;
            data = get(table,'Data');
            % Modify table data
            data{1,1} = sprintf('%.2f',knownVal);
            data{1,2} = sprintf('%.2f',measuredVal);
            % Absolute difference
            absDiff = abs(measuredVal-knownVal);
            data{1,3} = sprintf('%.2f',absDiff);
            % Percent difference
            avg = (knownVal+measuredVal)/2;
            percentDiff = absDiff/avg*100;
            data{1,4} = sprintf('%.2f',percentDiff);
            % Result
            if result == 1
                data{1,5} = '<html><font color="green">PASS';
            elseif result == 0
                data{1,5} = '<html><font color="red">FAIL';
            end
            % Set table data
            set(table,'Data',data);
            
            % Show scale warning if needed
            showScaleWarning(hObject,handles);
            % Get updated handles
            handles = guidata(hObject);
            
            % Store image for export
            im = getframe(handles.area_axes);
            handles.exportImages{testNum}{1} = im.cdata;
        end
    end
catch exception
    disp(getReport(exception));
end

% Set cursor back to normal
set(handles.figure1,'pointer',oldpointer);

guidata(hObject,handles);


% --- Executes on button press in area_button_setKnownVal.
function button_setKnownVal_Callback(hObject, eventdata, handles)
% hObject    handle to area_button_setKnownVal (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Get test name
testName = handles.testName;
switch testName
    case 'axialDistance'
        str = 'axial distance';
        baselineText = 'Axial distance';
    case 'lateralDistance'
        str = 'lateral distance';
        baselineText = 'Lateral distance';
    case 'area'
        str = 'area';
        baselineText = 'Area';
    case 'volume'
        str = 'volume';
        baselineText = 'Volume';
    case 'volumeFormula'
        str = 'volume';
        baselineText = 'Volume';
end

% Manually input known value
prompt = ['Enter the known ' str ':'];
title = ['Manually input the known ' str];
numlines = [1, length(title)+20];
default = {num2str(handles.([testName '_knownVal']))};
answer=inputdlg(prompt,title,numlines,default);

% If user inputted number
if ~isempty(answer)
    knownVal = str2num(answer{1});
    % Save to baseline file
    [num,txt,baselineFile] = xlsread('Baseline.xls');
    for i = 1:size(baselineFile,1)
        if ~isempty(strfind(baselineFile{i,1},baselineText))
            oldVal = baselineFile{i,2};
            baselineFile{i,2} = knownVal;
        end
    end
    try
        xlswrite('Baseline.xls',baselineFile);
        save('Baseline.mat','baselineFile');
        % Input in table
        if strcmp(testName,'lateralDistance')
            % If test is lateral distance, get axial or longitudinal table
            if strcmp(handles.([testName '_plane']),'axial')
                table = handles.([testName '_table_axial']);
            else
                table = handles.([testName '_table_long']);
            end
        else
            table = handles.([testName '_table']);
        end
        data = get(table,'Data');
        % If new known value is different, clear table
        if oldVal ~= knownVal
            data = cell(size(data));
        end
        % Display new known value in first column
        data(:,1) = {sprintf('%.2f',knownVal)};
        set(table,'Data',data);
        % Update handles. property
        handles.([testName '_knownVal']) = knownVal;
    catch
        warndlg('Unable to write to the baseline file. The file may be open in another application.',...
            'Warning');
    end
end
guidata(hObject,handles);


% --- Executes during object creation, after setting all properties.
function volume_table_CreateFcn(hObject, eventdata, handles)
% hObject    handle to volume_table (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
data = cell(1,5);
set(hObject,'Data',data);
set(hObject,'RowName',{'Volume'});
set(hObject,'ColumnName',{'<html>Known (cm<sup>3</sup>)</html>','<html>Measured (cm<sup>3</sup>)</html>',...
    'Diff (abs)','Diff (%)','Result'});
set(hObject,'ColumnEditable',false(1,size(data,2)));


% --- Executes on button press in volume_button_runTest.
function volume_button_runTest_Callback(hObject, eventdata, handles)
% hObject    handle to volume_button_runTest (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Get test number and name (test function called and gui handles depend on this name)

% Show loading cursor
oldpointer = get(handles.figure1,'pointer');
set(handles.figure1,'pointer','watch') 
drawnow;

try
    testNum = handles.testNum;
    panelHandle = handles.volume_panel_figure;
    axesHandles = handles.volume_axes_list;
    
    if numel(handles.images) >= testNum
        if ~isempty(handles.images{testNum})
            
            % Clear existing grid view
            delete(get(panelHandle,'Children'));
            % Clear existing axes
            for a = 1:numel(axesHandles)
                cla(axesHandles(a));
            end
            % Remove old legends
            parentPanel = get(axesHandles(1),'Parent');
            legends = findobj(get(parentPanel,'Children'),'Type','legend');
            delete(legends);
            % Clear 3D view axes
            cla(handles.volume_axes_interpView);
            cla(handles.volume_axes_slicesView);
            % Bring panel_figure back on top
            uistack(panelHandle,'top');
            
            % Run test, plot on given axes
            % Check if scale readings were set manually
            if ~isempty(handles.upperScaleReading{testNum}) && ~isempty(handles.lowerScaleReading{testNum})
                % Scale readings were inputted
                [result,knownVal,measuredVal] = volumeTestAuto(handles.images{testNum}{:},...
                    'UpperScale',handles.upperScaleReading{testNum},'LowerScale',handles.lowerScaleReading{testNum},...
                    'PanelHandle',panelHandle,'AxesHandle',axesHandles,...
                    'SlicesAxes',handles.volume_axes_slicesView,'InterpAxes',handles.volume_axes_interpView);
            else
                % Read scale automatically from image
                [result,knownVal,measuredVal] = volumeTestAuto(handles.images{testNum}{:},...
                    'PanelHandle',panelHandle,'AxesHandle',axesHandles,...
                    'SlicesAxes',handles.volume_axes_slicesView,'InterpAxes',handles.volume_axes_interpView);
            end
            
            % Get updated handles
            handles = guidata(hObject);
            
            % Get table data
            table = handles.volume_table;
            data = get(table,'Data');
            % Modify table data
            data{1,1} = sprintf('%.2f',knownVal);
            data{1,2} = sprintf('%.2f',measuredVal);
            % Absolute difference
            absDiff = abs(measuredVal-knownVal);
            data{1,3} = sprintf('%.2f',absDiff);
            % Percent difference
            avg = (knownVal+measuredVal)/2;
            percentDiff = absDiff/avg*100;
            data{1,4} = sprintf('%.2f',percentDiff);
            % Result
            if result == 1
                data{1,5} = '<html><font color="green">PASS';
            elseif result == 0
                data{1,5} = '<html><font color="red">FAIL';
            end
            % Set table data
            set(table,'Data',data);
            
            % If in single view mode, hide the grid panel and show current axes
            if get(handles.volume_button_singleView,'Value') == 1
                % Hide grid panel
                set(handles.volume_panel_figure,'Visible','off');
                % Show current image axes plots
                imageIndex = handles.volume_imageIndex;
                currImageAxes = handles.volume_axes_list(imageIndex);
                plots = get(currImageAxes,'Children');
                set(plots,'Visible','on');
                % Show legend associated with current image
                testPanelChildren = get(handles.volume_panel,'Children');
                legends = findobj(testPanelChildren,'Type','legend');
                for n = 1:numel(legends)
                    leg = legends(n);
                    userData = get(leg,'UserData');
                    if userData.ImageIndex == imageIndex
                        set(leg,'Visible','on');
                    end
                end
                % Show previous and next image buttons
                set(handles.volume_button_prev,'Visible','on');
                set(handles.volume_button_next,'Visible','on');
            end
            
            % Show selected 3D view and hide the other view
            switch handles.volume_3DView
                case 'interp'
                    set(get(handles.volume_axes_slicesView,'Children'),'Visible','off');
                    set(get(handles.volume_axes_interpView,'Children'),'Visible','on');
                case 'slices'
                    set(get(handles.volume_axes_interpView,'Children'),'Visible','off');
                    set(get(handles.volume_axes_slicesView,'Children'),'Visible','on');
            end
            
            % Show scale warning if needed
            showScaleWarning(hObject,handles);
            % Get updated handles
            handles = guidata(hObject);
            
            % Store images for export
            for a = 1:numel(handles.volume_axes_list)
                ax = handles.volume_axes_list(a);
                im = getframe(ax);
                handles.exportImages{testNum}{a} = im.cdata;
            end
        end
    end
catch exception
    disp(getReport(exception));
end

% Set cursor back to normal
set(handles.figure1,'pointer',oldpointer)

guidata(hObject,handles);


% --- Executes during object creation, after setting all properties.
function gridAlignment_table_CreateFcn(hObject, eventdata, handles)
% hObject    handle to gridAlignment_table (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
data = cell(1,2);
set(hObject,'Data',data);
set(hObject,'RowName',{'  '});
set(hObject,'ColumnName',{'Error (mm)','Result'});
set(hObject,'ColumnEditable',false(1,size(data,2)));


% --- Executes on button press in gridAlignment_button_runTest.
function gridAlignment_button_runTest_Callback(hObject, eventdata, handles)
% hObject    handle to gridAlignment_button_runTest (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Get test number and name (test function called and gui handles depend on this name)

% Show loading cursor
oldpointer = get(handles.figure1,'pointer');
set(handles.figure1,'pointer','watch') 
drawnow;

try
    if ~any(cellfun(@isempty,handles.gridAlignmentCoords))
        % Only run test if grid coords have been assigned for each image
        testNum = handles.testNum;
        panelHandle = handles.gridAlignment_panel_figure;
        axesHandles = handles.gridAlignment_axes_list;
        
        if numel(handles.images) >= testNum
            if ~isempty(handles.images{testNum})
                
                % Clear existing grid view
                delete(get(panelHandle,'Children'));
                % Clear existing axes
                for a = 1:numel(axesHandles)
                    cla(axesHandles(a));
                end
                % Remove old legends
                parentPanel = get(axesHandles(1),'Parent');
                legends = findobj(get(parentPanel,'Children'),'Type','legend');
                delete(legends);
                % Bring panel_figure back on top
                uistack(panelHandle,'top');
                
                % Run test, plot on given axes
                % Check if scale readings were set manually
                if ~isempty(handles.upperScaleReading{testNum}) && ~isempty(handles.lowerScaleReading{testNum})
                    % Scale readings were inputted
                    [result,errors,probeToG2] = gridAlignmentTestAuto(handles.images{testNum}{:},...
                        'UpperScale',handles.upperScaleReading{testNum},'LowerScale',handles.lowerScaleReading{testNum},...
                        'GridCoords',handles.gridAlignmentCoords,...
                        'PanelHandle',panelHandle,'AxesHandle',axesHandles);
                else
                    % Read scale automatically from image
                    [result,errors,probeToG2] = gridAlignmentTestAuto(handles.images{testNum}{:},...
                        'GridCoords',handles.gridAlignmentCoords,...
                        'PanelHandle',panelHandle,'AxesHandle',axesHandles);
                end
                
                % Get updated handles
                handles = guidata(hObject);
                
                % Get table data
                table = handles.gridAlignment_table;
                data = get(table,'Data');
                for n = 1:numel(errors)
                    % Modify table data
                    data{n,1} = sprintf('%.2f',errors(n));
                    % Result
                    if result(n) == 1
                        data{n,2} = '<html><font color="green">PASS';
                    else
                        data{n,2} = '<html><font color="red">FAIL';
                    end
                end
                % Set table data
                set(table,'Data',data);
                
                % Set Probe to G2 table data
                table2 = handles.gridAlignment_table_probeToG2;
                data2 = get(table2,'Data');
                data2{1} = sprintf('%.2f',probeToG2);
                set(table2,'Data',data2);
                
                % If in single view mode, hide the grid panel and show current axes
                if get(handles.gridAlignment_button_singleView,'Value') == 1
                    % Hide grid panel
                    set(handles.gridAlignment_panel_figure,'Visible','off');
                    % Show current image axes plots
                    imageIndex = handles.gridAlignment_imageIndex;
                    currImageAxes = handles.gridAlignment_axes_list(imageIndex);
                    plots = get(currImageAxes,'Children');
                    set(plots,'Visible','on');
                    % Show legend associated with current image
                    testPanelChildren = get(handles.gridAlignment_panel,'Children');
                    legends = findobj(testPanelChildren,'Type','legend');
                    for n = 1:numel(legends)
                        leg = legends(n);
                        userData = get(leg,'UserData');
                        if userData.ImageIndex == imageIndex
                            set(leg,'Visible','on');
                        end
                    end
                    % Show previous and next image buttons
                    set(handles.gridAlignment_button_prev,'Visible','on');
                    set(handles.gridAlignment_button_next,'Visible','on');
                end
                
                % Show scale warning if needed
                showScaleWarning(hObject,handles);
                % Get updated handles
                handles = guidata(hObject);
            end
        end
    end
catch exception
    disp(getReport(exception));
end

% Set cursor back to normal
set(handles.figure1,'pointer',oldpointer);

guidata(hObject,handles);


% --- Executes on button press in depth_button_setBaseline.
function button_setBaseline_Callback(hObject, eventdata, handles)
% hObject    handle to depth_button_setBaseline (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
testName = handles.testName;
msg = '';

switch testName
    case 'grayscale'
        str = 'grayscale';
        baselineText = 'Grayscale';
    case 'depth'
        str = 'depth';
        baselineText = 'Depth';
    case 'axialResolution'
        str = 'axial resolution';
        baselineText = 'Axial resolution';
    case 'lateralResolution'
        str = 'lateral resolution';
        baselineText = 'Lateral resolution';
end

if ~strcmp(testName,'grayscale')
    % Get axial data
    table_axial = handles.([testName '_table_axial']);
    data_axial = get(table_axial,'Data');
    strVals_axial = data_axial(:,2);
    if ~isempty(strVals_axial)
        currVals_axial = cell(size(strVals_axial));
        for n = 1:numel(strVals_axial)
            if ~isempty(strVals_axial{n})
                rowName = get(table_axial,'RowName');
                msg = sprintf([msg '\n' rowName{n} ' (axial plane): ' strVals_axial{n}]);
                % Convert values to double format before writing to file
                currVals_axial{n} = str2num(strVals_axial{n});
            end
        end
    end
    % Get longitudinal data
    table_long = handles.([testName '_table_long']);
    data_long = get(table_long,'Data');
    strVals_long = data_long(:,2);
    if ~isempty(strVals_long)
        currVals_long = cell(size(strVals_long));
        for n = 1:numel(strVals_long)
            if ~isempty(strVals_long{n})
                rowName = get(table_long,'RowName');
                msg = sprintf([msg '\n' rowName{n} ' (longitudinal plane): ' strVals_long{n}]);
                % Convert values to double format before writing to file
                currVals_long{n} = str2num(strVals_long{n});
            end
        end
    end
    % Combine data
    currVals = [currVals_axial;currVals_long]';
else
    % Grayscale test doesnt have separate axial and longitudinal tables
    table = handles.([testName '_table']);
    data = get(table,'Data');
    strVals = data(:,2);
    if ~isempty(strVals)
        currVals = cell(size(strVals));
        for n = 1:numel(strVals)
            if ~isempty(strVals{n})
                rowName = get(table,'RowName');
                msg = sprintf([msg '\n' rowName{n} ': ' strVals{n}]);
                % Convert values to double format before writing to file
                currVals{n} = str2num(strVals{n});
            end
        end
    end
end

% Confirm overwrite of baseline value
choice = questdlg(sprintf(['Overwrite baseline values with the current measurements? '...
    '\n' 'The following values will be written to the baseline file:'...
    '\n' msg]), ...
    'Confirm Baseline Overwrite', ...
    'Yes','No','Yes');

if strcmp(choice,'Yes')
    % Save to baseline file
    [num,txt,baselineFile] = xlsread('Baseline.xls');
    for i = 1:size(baselineFile,1)
        if ~isempty(strfind(baselineFile{i,1},baselineText))
            for n = 1:numel(currVals)
                % Only write new value if not empty
                if ~isempty(currVals{n})
                    baselineFile{i,1+n} = currVals{n};
                end
            end
            %             baselineFile(i,2:numel(currVals)+1) = currVals;
        end
    end
    try
        xlswrite('Baseline.xls',baselineFile);
        save('Baseline.mat','baselineFile');
    catch
        errordlg('Unable to write to the baseline file. The file may be open in another application.',...
            'Error');
    end
end


% --- Executes on mouse press over axes background.
function grayscale_axes_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to grayscale_axes (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
switch get(hObject,'SelectionType')
    case 'normal'
    case 'open'
        disp('double click')
end

% --- Shows warning dialog about assumed pixel scale
function showScaleWarning(hObject, handles)
if handles.AssumedScale == 1 && handles.ShowScaleWarning == 1
    msg = sprintf(['Unable to read scale from image. The following values have been assumed:'...
        '\n\n' 'Upper bound: 6.4 cm'...
        '\n' 'Lower bound: 0 cm'...
        '\n\n' 'If these are not the correct values, please input the scale readings manually.'],...
        'Warning');
    
    h = msgbox({msg,' ',' ',' '}, 'Warning','Warn','modal');
    checkbox = uicontrol('Style','checkbox','String','Don''t show this message again',...
        'Parent',h,'Units','normalized','Position',[0.3 0.25 0.8 0.1]);

    % Wait for user to press OK button, see if checkbox was checked
    while ishandle(h)
        if get(checkbox,'Value') == 1
            % Set flag so that this warning message doesn't appear anymore
            handles.ShowScaleWarning = 0;
        else
            % Checkbox wasn't clicked, continue to show warning if needed
            handles.ShowScaleWarning = 1;
        end
        pause(0.1);
    end
    
    % Reset AssumedScale flag to 0
    handles.AssumedScale = 0;
    % Update handles
    guidata(hObject,handles);
    
end


% --- Executes on button press in button_flipHor.
function button_flipHor_Callback(hObject, eventdata, handles)
% hObject    handle to gridAlignment_button_flipHor (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
testNum = handles.testNum;
testName = handles.testName;

% If images have been selected
if ~isempty(handles.images{testNum})
    if ~any(strcmp(testName,{'volume','volumeFormula','gridAlignment'}))
        % For any test other than volume or grid alignment
        % Flip image horizontally
        flippedIm = fliplr(handles.images{testNum}{1});
        handles.images{testNum}{1} = flippedIm;
        
        % Delete axes plots other than image
        axesHandle = handles.([testName '_axes']);
        otherPlots = findobj(get(axesHandle,'Children'),'-not','Type','image');
        delete(otherPlots);
        % Remove old legends
        parentPanel = get(axesHandle,'Parent');
        legends = findobj(get(parentPanel,'Children'),'Type','legend');
        delete(legends);
        
        % Update image 
        imPlot = findobj(get(axesHandle,'Children'),'Type','image');
        set(imPlot,'CData',flippedIm);
    else
        % For volume and grid alignment tests
        % Delete plots and legends other than axes and images on grid view
        gridPanel = handles.([testName '_panel_figure']);
        otherGrid = findobj(get(gridPanel,'Children'),'-not','Type','axes',...
            '-not','Type','image');
        delete(otherGrid);
        % Delete other plots on single view axes
        for n = 1:numel(handles.([testName '_axes_list']))
            axesHandle = handles.([testName '_axes_list'])(n);
            otherSingle = findobj(get(axesHandle,'Children'),'-not','Type','image');
            delete(otherSingle);
        end
        % Delete single view legends
        ax = handles.([testName '_axes_list'])(1);
        parent = get(ax,'Parent');
        legends = findobj(get(parent,'Children'),'Type','legend');
        delete(legends);
        
        if get(handles.([testName '_button_gridView']),'Value') == 1
            % Grid view - flip all images
            imPlots = findobj(get(gridPanel,'Children'),'Type','image');
            for i = 1:numel(imPlots)
                imageIndex = get(imPlots(i),'UserData');
                % Flip image
                flippedIm = fliplr(get(imPlots(i),'CData'));
                % Update plotted grid view image
                set(imPlots(i),'CData',flippedIm);
                
                % Update stored image in handles
                handles.images{testNum}{imageIndex} = flippedIm;
                % Update corresponding single view image
                axesHandle = handles.([testName '_axes_list'])(imageIndex);
                imSingle = findobj(get(axesHandle,'Children'),'Type','image');
                set(imSingle,'CData',flippedIm);
            end
        else
            % Single view - flip current image
            imageIndex = handles.([testName '_imageIndex']);
            axesHandle = handles.([testName '_axes_list'])(imageIndex);
            imSingle = findobj(get(axesHandle,'Children'),'Type','image');
            % Flip image
            flippedIm = fliplr(get(imSingle,'CData'));
            % Update stored image in handles
            handles.images{testNum}{imageIndex} = flippedIm;
            % Update plotted single view image
            set(imSingle,'CData',flippedIm);
            
            % Update corresponding grid view image
            gridImage = findobj(get(gridPanel,'Children'),'Type','image',...
                'UserData',imageIndex);
            set(gridImage,'CData',flippedIm);
        end
    end
    % Update handles
    guidata(hObject,handles);
end
    

% --- Executes on button press in button_flipVert.
function button_flipVert_Callback(hObject, eventdata, handles)
% hObject    handle to gridAlignment_button_flipVert (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
testNum = handles.testNum;
testName = handles.testName;

% If images have been selected
if ~isempty(handles.images{testNum})
    if ~any(strcmp(testName,{'volume','volumeFormula','gridAlignment'}))
        % For any test other than volume or grid alignment
        % Flip image vertically
        flippedIm = flipud(handles.images{testNum}{1});
        handles.images{testNum}{1} = flippedIm;
        
        % Delete axes plots other than image
        axesHandle = handles.([testName '_axes']);
        otherPlots = findobj(get(axesHandle,'Children'),'-not','Type','image');
        delete(otherPlots);
        % Remove old legends
        parentPanel = get(axesHandle,'Parent');
        legends = findobj(get(parentPanel,'Children'),'Type','legend');
        delete(legends);
        
        % Update image 
        imPlot = findobj(get(axesHandle,'Children'),'Type','image');
        set(imPlot,'CData',flippedIm);
    else
        % For volume and grid alignment tests
        % Delete plots and legends other than axes and images on grid view
        gridPanel = handles.([testName '_panel_figure']);
        otherGrid = findobj(get(gridPanel,'Children'),'-not','Type','axes',...
            '-not','Type','image');
        delete(otherGrid);
        % Delete other plots on single view axes
        for n = 1:numel(handles.([testName '_axes_list']))
            axesHandle = handles.([testName '_axes_list'])(n);
            otherSingle = findobj(get(axesHandle,'Children'),'-not','Type','image');
            delete(otherSingle);
        end
        % Delete single view legends
        ax = handles.([testName '_axes_list'])(1);
        parent = get(ax,'Parent');
        legends = findobj(get(parent,'Children'),'Type','legend');
        delete(legends);
        
        if get(handles.([testName '_button_gridView']),'Value') == 1
            % Grid view - flip all images
            imPlots = findobj(get(gridPanel,'Children'),'Type','image');
            for i = 1:numel(imPlots)
                imageIndex = get(imPlots(i),'UserData');
                % Flip image
                flippedIm = flipud(get(imPlots(i),'CData'));
                % Update plotted grid view image
                set(imPlots(i),'CData',flippedIm);
                
                % Update stored image in handles
                handles.images{testNum}{imageIndex} = flippedIm;
                % Update corresponding single view image
                axesHandle = handles.([testName '_axes_list'])(imageIndex);
                imSingle = findobj(get(axesHandle,'Children'),'Type','image');
                set(imSingle,'CData',flippedIm);
            end
        else
            % Single view - flip current image
            imageIndex = handles.([testName '_imageIndex']);
            axesHandle = handles.([testName '_axes_list'])(imageIndex);
            imSingle = findobj(get(axesHandle,'Children'),'Type','image');
            % Flip image
            flippedIm = flipud(get(imSingle,'CData'));
            % Update stored image in handles
            handles.images{testNum}{imageIndex} = flippedIm;
            % Update plotted single view image
            set(imSingle,'CData',flippedIm);
            
            % Update corresponding grid view image
            gridImage = findobj(get(gridPanel,'Children'),'Type','image',...
                'UserData',imageIndex);
            set(gridImage,'CData',flippedIm);
        end
    end
    % Update handles
    guidata(hObject,handles);
end


% --- Executes on mouse press over figure background, over a disabled or
% --- inactive control, or over an axes background.
function figure1_WindowButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
testName = handles.testName;
switch testName
    case 'volume'
        axes3D = handles.volume_axes_slicesView;
    case 'volumeFormula'
        axes3D = handles.volumeFormula_axes_ellipsoidView;
    otherwise
        axes3D = [];
end
% Only rotate if on volume (planimetric) or volume (formula) test
if ~isempty(axes3D)
    % Get position of axes relative to figure
    set(axes3D,'Units','normalized');
    axPos = get(axes3D,'Position');
    parent = get(axes3D,'Parent');
    set(parent,'Units','normalized');
    parentPos = get(parent,'Position');
    pos = [axPos(1)+parentPos(1) axPos(2)+parentPos(2) axPos(3) axPos(4)];
    
    % Get the coordinates of the mouse click
    mousePoint = get(hObject,'CurrentPoint');
    % Check if clicked within 3D plot axes
    if (mousePoint(1) > pos(1) && mousePoint(1) < pos(1)+pos(3)) && ...
            (mousePoint(2) > pos(2) && mousePoint(2) < pos(2)+pos(4))
        % Clicked within slices axes, set rotate flag to on
        handles.rotate3D = 1;
    end
    % Update handles
    guidata(hObject,handles);
end


% --- Executes on mouse press over figure background, over a disabled or
% --- inactive control, or over an axes background.
function figure1_WindowButtonUpFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Reset rotate flag
handles.rotate3D = 0;
% Reset old mouse point (the first old mouse point should always be the
% point when first clicking)
handles.oldMousePoint = [];
% Update handles
guidata(hObject,handles);


% --- Executes on mouse motion over figure - except title and menu.
function figure1_WindowButtonMotionFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Check if 3D view axes was clicked on, rotate flag is set
if handles.rotate3D == 1
    switch handles.testName
        case 'volume'
            axesHandles = [handles.volume_axes_interpView;handles.volume_axes_slicesView];
        case 'volumeFormula'
            axesHandles = handles.volumeFormula_axes_ellipsoidView;
    end
    % Get coordinates of new mouse point
    newMousePoint = get(hObject,'CurrentPoint');
    % If user just clicked, initialize oldMousePoint
    if isempty(handles.oldMousePoint)
        handles.oldMousePoint = newMousePoint;
    end
    % Rotate camera
    switch get(hObject,'Units')
        case 'normalized'
            multiplier = 200;
        case 'characters'
            multiplier = 2;
    end
    dtheta = -(newMousePoint(1)-handles.oldMousePoint(1));
    dphi = -(newMousePoint(2)-handles.oldMousePoint(2));
    
    for a = 1:numel(axesHandles)
        camorbit(axesHandles(a),multiplier*dtheta,multiplier*dphi);
    end
    
    % Store old mouse point
    handles.oldMousePoint = newMousePoint;
end
% Update handles
guidata(hObject,handles);


% --- Executes when selected object is changed in volume_buttongroup_3DView.
function volume_buttongroup_3DView_SelectionChangeFcn(hObject, eventdata, handles)
% hObject    handle to the selected object in volume_buttongroup_3DView 
% eventdata  structure with the following fields (see UIBUTTONGROUP)
%	EventName: string 'SelectionChanged' (read only)
%	OldValue: handle of the previously selected object or empty if none was selected
%	NewValue: handle of the currently selected object
% handles    structure with handles and user data (see GUIDATA)
selection = get(eventdata.NewValue,'Tag');
switch selection
    case 'volume_button_interpView'
        handles.volume_3DView = 'interp';
        % Show interp view plot, hide others
        set(get(handles.volume_axes_slicesView,'Children'),'Visible','off');
        set(get(handles.volume_axes_interpView,'Children'),'Visible','on');
    case 'volume_button_slicesView'
        handles.volume_3DView = 'slices';
        % Show slices view plot, hide others
        set(get(handles.volume_axes_interpView,'Children'),'Visible','off');
        set(get(handles.volume_axes_slicesView,'Children'),'Visible','on');
end
% Update handles
guidata(hObject,handles);
        


% --- Executes on button press in volumeFormula_button_runTest.
function volumeFormula_button_runTest_Callback(hObject, eventdata, handles)
% hObject    handle to volumeFormula_button_runTest (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Show loading cursor
oldpointer = get(handles.figure1,'pointer');
set(handles.figure1,'pointer','watch') 
drawnow;

try
    testNum = handles.testNum;
    panelHandle = handles.volumeFormula_panel_figure;
    axesHandles = handles.volumeFormula_axes_list;
    
    if numel(handles.images) >= testNum
        if ~isempty(handles.images{testNum})
            
            % Clear existing grid view
            delete(get(panelHandle,'Children'));
            % Clear existing axes
            for a = 1:numel(axesHandles)
                cla(axesHandles(a));
            end
            % Remove old legends
            parentPanel = get(axesHandles(1),'Parent');
            legends = findobj(get(parentPanel,'Children'),'Type','legend');
            delete(legends);
            % Clear 3D view axes
            cla(handles.volumeFormula_axes_ellipsoidView);
            % Bring panel_figure back on top
            uistack(panelHandle,'top');
            
            % Run test, plot on given axes
            % Check if scale readings were set manually
            if ~isempty(handles.upperScaleReading{testNum}) && ~isempty(handles.lowerScaleReading{testNum})
                % Scale readings were inputted
                [result,knownVal,measuredVal] = volumeTestFormula(handles.images{testNum}{:},...
                    'UpperScaleAxial',handles.upperScaleReading{testNum}{1},'LowerScaleAxial',handles.lowerScaleReading{testNum}{1},...
                    'UpperScaleSagittal',handles.upperScaleReading{testNum}{2},'LowerScaleSagittal',handles.lowerScaleReading{testNum}{2},...
                    'PanelHandle',panelHandle,'AxesHandle',axesHandles,...
                    'EllipsoidAxes',handles.volumeFormula_axes_ellipsoidView);
            else
                % Read scale automatically from image
                [result,knownVal,measuredVal] = volumeTestFormula(handles.images{testNum}{:},...
                    'PanelHandle',panelHandle,'AxesHandle',axesHandles,...
                    'EllipsoidAxes',handles.volumeFormula_axes_ellipsoidView);
            end
            
            % Get updated handles
            handles = guidata(hObject);
            
            % Get table data
            table = handles.volumeFormula_table;
            data = get(table,'Data');
            % Modify table data
            data{1,1} = sprintf('%.2f',knownVal);
            data{1,2} = sprintf('%.2f',measuredVal);
            % Absolute difference
            absDiff = abs(measuredVal-knownVal);
            data{1,3} = sprintf('%.2f',absDiff);
            % Percent difference
            avg = (knownVal+measuredVal)/2;
            percentDiff = absDiff/avg*100;
            data{1,4} = sprintf('%.2f',percentDiff);
            % Result
            if result == 1
                data{1,5} = '<html><font color="green">PASS';
            elseif result == 0
                data{1,5} = '<html><font color="red">FAIL';
            end
            % Set table data
            set(table,'Data',data);
            
            % If in single view mode, hide the grid panel and show current axes
            if get(handles.volumeFormula_button_singleView,'Value') == 1
                % Hide grid panel
                set(handles.volumeFormula_panel_figure,'Visible','off');
                % Show current image axes plots
                imageIndex = handles.volumeFormula_imageIndex;
                currImageAxes = handles.volumeFormula_axes_list(imageIndex);
                plots = get(currImageAxes,'Children');
                set(plots,'Visible','on');
                % Show legend associated with current image
                testPanelChildren = get(handles.volumeFormula_panel,'Children');
                legends = findobj(testPanelChildren,'Type','legend');
                for n = 1:numel(legends)
                    leg = legends(n);
                    userData = get(leg,'UserData');
                    if userData.ImageIndex == imageIndex
                        set(leg,'Visible','on');
                    end
                end
                % Show previous and next image buttons
                set(handles.volumeFormula_button_prev,'Visible','on');
                set(handles.volumeFormula_button_next,'Visible','on');
            end
            
            % Show scale warning if needed
            showScaleWarning(hObject,handles);
            % Get updated handles
            handles = guidata(hObject);
        end
    end
catch exception
    disp(getReport(exception));
end

% Set cursor back to normal
set(handles.figure1,'pointer',oldpointer)

guidata(hObject,handles);

% --- Executes on button press in volumeFormula_button_setScale.
function volumeFormula_button_setScale_Callback(hObject, eventdata, handles)
% hObject    handle to volumeFormula_button_setScale (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get test number
testNum = handles.testNum;

% Manually input upper and lower scale readings
prompt = {['\bf Axial Image: ','\rm Enter the upper scale reading (cm):'],...
    ['\bf Axial Image: ','\rm Enter the lower scale reading (cm):'],...
    ['\bf Sagittal Image: ','\rm Enter the upper scale reading (cm):'],...
    ['\bf Sagittal Image: ','\rm Enter the lower scale reading (cm):']};
title = 'Manually input scale readings (optional)';
numlines = [1, length(title)+20];

% Default values
defaultAxialUpper = num2str(handles.upperScaleReading{testNum}{1});
defaultAxialLower = num2str(handles.lowerScaleReading{testNum}{1});
defaultSagittalUpper = num2str(handles.upperScaleReading{testNum}{2});
defaultSagittalLower = num2str(handles.lowerScaleReading{testNum}{2});
default = {defaultAxialUpper,defaultAxialLower,defaultSagittalUpper,defaultSagittalLower};

options.Interpreter = 'tex';
answer=inputdlg(prompt,title,numlines,default,options);
% If user inputted both numbers
if ~isempty(answer)
    handles.upperScaleReading{testNum}{1} = str2num(answer{1});
    handles.lowerScaleReading{testNum}{1} = str2num(answer{2});
    handles.upperScaleReading{testNum}{2} = str2num(answer{3});
    handles.lowerScaleReading{testNum}{2} = str2num(answer{4});
end
guidata(hObject,handles);


% --- Executes on button press in phantom_button_addField.
function phantom_button_addField_Callback(hObject, eventdata, handles)
% hObject    handle to phantom_button_addField (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Manually input upper and lower scale readings
prompt = 'Enter the new field name:';
title = 'Add Field';
numlines = [1, length(title)+20];
answer=inputdlg(prompt,title,numlines);
% If user inputted new field name
if ~isempty(answer)
    % Add new field to existing row headers
    fields = get(handles.phantom_table,'RowName');
    newFields = [fields;answer];
    set(handles.phantom_table,'RowName',newFields);
    % Add new entry for table data
    tableData = get(handles.phantom_table,'Data');
    newTableData = [tableData;cell(1)];
    set(handles.phantom_table,'Data',newTableData);
%     set(handles.phantom_table,'ColumnEditable',true);
end
guidata(hObject,handles);

% --- Executes on button press in phantom_button_delField.
function phantom_button_delField_Callback(hObject, eventdata, handles)
% hObject    handle to phantom_button_delField (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Create popup dialog
width = 300;
height = 200;
screenSize = get(0,'screensize');
x = round(screenSize(3)/2 - width/2);
y = round(screenSize(4)/2 - height/2);
d = dialog('Name','Delete Fields','Position',[x y width height]);
% set(d,'WindowStyle','normal');
% Create instructions static text
msg = uicontrol('Parent',d,'Style','text','String','Select field(s) to delete:',...
    'HorizontalAlignment','left','Position',[10 180 280 15]);
msgPos = get(msg,'Position');
% Create listbox showing fields
listbox = uicontrol('Parent',d,'Style','listbox',...
    'Position',[msgPos(1) 50 msgPos(3) 125]);
fields = get(handles.phantom_table,'RowName');
set(listbox,'String',fields);
% Set Max property so multiple fields can be selected
set(listbox,'Max',numel(fields));
% Create delete button and set callback
deleteBtn = uicontrol('Parent',d,'String','Delete','Position',[100 10 50 30],...
    'Callback',@(obj,eventdata)deleteFields(listbox,handles));
% Create cancel button and set callback
cancelBtn = uicontrol('Parent',d,'String','Cancel','Position',[160 10 50 30],...
    'Callback','delete(gcf)');
guidata(hObject,handles);

function deleteFields(listbox,handles)
% Get indices of selected fields
selected = get(listbox,'Value');
% Delete selected fields
fields = get(handles.phantom_table,'RowName');
fields(selected) = [];
% Delete selected fields from table row headers
set(handles.phantom_table,'RowName',fields);
% Delete selected fields from listbox
set(listbox,'Value',1);
set(listbox,'String',fields);
% Delete selected field rows from table data
tableData = get(handles.phantom_table,'Data');
tableData(selected) = [];
set(handles.phantom_table,'Data',tableData);
% Close the Delete Fields popup window
delete(gcf);


% --- Executes on button press in phantom_button_saveDefFields.
function phantom_button_saveDefFields_Callback(hObject, eventdata, handles)
% hObject    handle to phantom_button_saveDefFields (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
fields = get(handles.phantom_table,'RowName');
save('DefaultFields.mat','fields');
msgbox('Default fields saved.');



% --- Executes during object creation, after setting all properties.
function phantom_table_CreateFcn(hObject, eventdata, handles)
% hObject    handle to phantom_table (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Use default fields in Default Fields mat file if created
if exist('DefaultFields.mat','file')
    load('DefaultFields.mat','fields');
else
    % Read excel file, check existing default fields
    filename = fullfile(pwd,'Log/','ProstateBrachyQA Log.xlsx');
    sheet = 1;
    % Read excel file
    [num,txt,excelData] = xlsread(filename,sheet);
    % Get field names
    fields = excelData(1,:);
    fields = fields(cellfun(@ischar,fields));
    % If sheet is empty (no field names), use default field 'Weight'
    if isempty(fields)
        fields = {'Weight (g)'};
    end
end

tableData = cell(numel(fields),1);
set(hObject,'Data',tableData);
set(hObject,'RowName',fields);
set(hObject,'ColumnName',[]);
set(hObject,'ColumnEditable',true);


% --- Executes on button press in gridAlignment_button_assignCoords.
function gridAlignment_button_assignCoords_Callback(hObject, eventdata, handles)
% hObject    handle to gridAlignment_button_assignCoords (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Manually input upper and lower scale readings
images = handles.images{handles.testNum};
% Create popup dialog
width = 350;
height = 450;
screenSize = get(0,'screensize');
x = round(screenSize(3)/2 - width/2);
y = round(screenSize(4)/2 - height/2);
d = dialog('Name','Assign Grid Coordinates','Position',[x y width height]);
set(d,'WindowStyle','normal');
% Create presets popup menu
str = uicontrol('Parent',d,'Style','text','String','Preset:',...
    'Position',[20 height-40 width-40 20],'HorizontalAlignment','left',...
    'FontWeight','bold');
% Load presets if file exists, otherwise create new presets
if exist('GridCoordinatePresets.mat','file')
    load('GridCoordinatePresets.mat');
else
    presets{1} = 'B1,A3,A10,B12,L12,M10,M3,L1,H7,G7,F7,F6,G6,H6';
    presets{end+1} = 'Custom';
end
menu = uicontrol('Parent',d,'Style','popup','String',presets,...
    'Position',[20 height-50 width-40 10]);

% Select previously chosen preset
if ~isempty(handles.coordsPresetNum)
    set(menu,'Value',handles.coordsPresetNum);
    presetCell = handles.gridAlignmentCoords;
else
    % Get selected preset string
    choice = presets{get(menu,'Value')};
    % Convert selected preset to cell array
    switch choice
        case 'Custom'
            % Empty table, allow user to input manually
            presetCell = {''};
        otherwise
            % Automatically fill table with preset coordinates
            presetCell = strsplit(choice,',');
    end
end

% Create grid coords table
data = cell(numel(images),1);
data(1:numel(presetCell)) = presetCell;
rowNames = cell(numel(images),1);
for n = 1:numel(images)
    rowNames{n} = ['Image ' num2str(n)];
end
tab = uitable('Parent',d,'RowName',rowNames,'ColumnName','Coordinates','Data',data,...
    'ColumnEditable',true,'Position',[20 50 width-40 height-160]);
set(tab,'CellEditCallback',@(obj,eventdata)updatePresetMenu(menu,tab));
% Set callback for popup menu
set(menu,'Callback',@(obj,eventdata)fillCoordsTable(menu,tab,handles));

% Create Save Preset button
savePresetBtn = uicontrol('Parent',d,'String','Save Preset','Position',[20 height-100 100 30],...
    'Callback',@(obj,eventdata)savePreset_Callback(menu,tab));
% Create Delete Preset button
delPresetBtn = uicontrol('Parent',d,'String','Delete Preset','Position',[130 height-100 100 30],...
    'Callback',@(obj,eventdata)delPreset_Callback(menu,tab));

% Create OK button and set callback
okBtn = uicontrol('Parent',d,'String','OK','Position',[width/2-55 10 50 30],...
    'Callback',@(obj,eventdata)coordsOK_Callback(menu,tab,hObject,handles));
% Create cancel button and set callback
cancelBtn = uicontrol('Parent',d,'String','Cancel','Position',[width/2+5 10 50 30],...
    'Callback','delete(gcf)');
guidata(hObject,handles);

function fillCoordsTable(menuHandle,tableHandle,handles)
% Get list of presets in menu
presets = get(menuHandle,'String');
% Get selected preset string
presetStr = presets{get(menuHandle,'Value')};
% Convert selected preset to cell array
switch presetStr
    case 'Custom'
        % Start with existing coords, whether empty or modified
        presetCell = handles.gridAlignmentCoords;
    otherwise
        % Automatically fill table with preset coordinates
        presetCell = strsplit(presetStr,',');
end
% Clear the current table
oldData = get(tableHandle,'Data');
tableData = cell(size(oldData));
set(tableHandle,'Data',cell(size(tableData)));
% Fill table with selected preset values
if numel(tableData) < numel(presetCell)
    tableData(1:end) = presetCell(1:numel(tableData));
else
    tableData(1:numel(presetCell)) = presetCell;
end
set(tableHandle,'Data',tableData);

function coordsOK_Callback(menuHandle,tableHandle,hObject,handles)
% Store selected preset number
handles.coordsPresetNum = get(menuHandle,'Value');
% Store grid coordinates
gridCoords = get(tableHandle,'Data');
handles.gridAlignmentCoords = gridCoords';
% If there is a different number of new assigned coords or they are 
% different than existing ones in table, clear any existing results
if numel(gridCoords) ~= numel(get(handles.gridAlignment_table,'RowName'))
    set(handles.gridAlignment_table,'Data',repmat({''},numel(gridCoords),1));
elseif ~all(strcmp(gridCoords,get(handles.gridAlignment_table,'RowName')))
    data = get(handles.gridAlignment_table,'Data');
    set(handles.gridAlignment_table,'Data',repmat({''},size(data)));
end
% Set results table row headers
set(handles.gridAlignment_table,'RowName',gridCoords);

% Update handles
guidata(hObject,handles)
% Close assign coords dialog
delete(gcf);

function updatePresetMenu(menuHandle,tableHandle)
presets = get(menuHandle,'String');
preset = presets{get(menuHandle,'Value')};
tableData = get(tableHandle,'Data');
% If preset is not already 'Custom', check if table data matches
if ~strcmp(preset,'Custom')
    presetCell = strsplit(preset,',');
    % If table was edited and does not match preset, switch menu to 'Custom'
    if ~all(strcmp(presetCell(1:numel(tableData)),tableData'))
        set(menuHandle,'Value',numel(presets));
    end
end

function savePreset_Callback(menuHandle,tableHandle)
choices = get(menuHandle,'String');
selected = choices{get(menuHandle,'Value')};
% If presets file does not exist, create new one
if ~exist('GridCoordinatePresets.mat','file')
    presets = choices;
    save('GridCoordinatePresets.mat','presets');
end
if ~strcmp(selected,'Custom')
    warndlg('This preset already exists.','Save Preset');
else
    newPreset = strjoin(get(tableHandle,'Data')',',');
    load('GridCoordinatePresets.mat');
    % Check if preset already exists
    if ~any(strcmp(presets,newPreset))
        presets{end} = newPreset;
        presets{end+1} = 'Custom';
        set(menuHandle,'String',presets);
        save('GridCoordinatePresets.mat','presets');
        msgbox('Preset saved.','Save Preset');
    else
        warndlg('This preset already exists.','Save Preset');
    end
end

function delPreset_Callback(menuHandle,tableHandle)
choices = get(menuHandle,'String');
selected = choices{get(menuHandle,'Value')};
% If presets file does not exist, create new one
if ~exist('GridCoordinatePresets.mat','file')
    presets = choices;
    save('GridCoordinatePresets.mat','presets');
end
if strcmp(selected,'Custom')
    warndlg('This preset has not yet been saved.','Delete Preset');
else
    load('GridCoordinatePresets.mat');
    ind = strcmp(presets,selected);
    confirm = questdlg({'Are you sure you want to delete preset',[selected ' ?']},...
        'Confirm Delete');
    if strcmp(confirm,'Yes')
        % Delete preset
        presets(ind) = [];
        % Update popup menu
        set(menuHandle,'String',presets);
        set(menuHandle,'Value',numel(presets));
        % Clear table
        tableData = get(tableHandle,'Data');
        set(tableHandle,'Data',repmat({''},size(tableData)));
        % Save presets to file
        save('GridCoordinatePresets.mat','presets');
        msgbox('Preset deleted.','Delete Preset');
    end
end


% --- Executes on button press in phantom_button_export.
function phantom_button_export_Callback(hObject, eventdata, handles)
% hObject    handle to phantom_button_export (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
fields = get(handles.phantom_table,'RowName');
tableData = get(handles.phantom_table,'Data');

try
    % Get handle to Excel COM Server
    Excel = actxserver('Excel.Application');
    Excel.DisplayAlerts = 0;
    % Open Workbook
    Workbooks = Excel.Workbooks;
    Workbook = Open(Workbooks,fullfile(pwd,'Log/','ProstateBrachyQA Log.xlsx'));
    
    if Workbook.ReadOnly == 0
        % Have write access to excel file
        % Get a handle to Sheets and select Sheet 1
        Sheets = Excel.ActiveWorkBook.Sheets;
        if Sheets.Count < 1
            Sheet = Sheets.Add([],get(Sheets,'Item',Sheets.Count));
        else
            Sheet = get(Sheets, 'Item', 1);
        end
        Sheet.Name = 'Phantom';
        Sheet.Activate;
        % Get number of last used row
        lastRow = Sheet.get('Cells').Find('*',Sheet.get('Cells',1,1),[],[],1,2);
        if ~isempty(lastRow)
            numRows = lastRow.Row;
        else
            % No data, create first row of headers
            % Put title in first cell
            Sheet.get('Cells',1,1).Value = 'Phantom';
            % Colour first cell yellow
            Sheet.get('Cells',1,1).Interior.ColorIndex = 6;
            headers = Sheet.get('Range',Sheet.get('Cells',1,2),Sheet.get('Cells',1,numel(fields)+1));
            headers.Value = fields;
            numRows = 1;
            % Set first row to bold
            Sheet.Range('1:1').Font.Bold = 1;
            % Freeze first row
            Sheet.Application.ActiveWindow.SplitRow = 1;
            Sheet.Application.ActiveWindow.FreezePanes = true;
            % Set first column to bold
            Sheet.Range('A:A').Font.Bold = 1;
        end
        numCols = Sheet.get('Cells').Find('*',Sheet.get('Cells',1,1),[],[],2,2).Column;
        xlData = Sheet.UsedRange.Value(1:numRows,1:numCols);
        
        % Write new data
        numRows = numRows + 1;
        % Write date
        Sheet.get('Cells',numRows,1).Value = date;
        % Write fields
        for n = 1:numel(fields)
            field = fields{n};
            val = tableData{n};
            [~,fieldCol] = find(strcmp(xlData(1,:),field));
            if ~isempty(fieldCol)
                % Column exists, write the value in new row
                Sheet.get('Cells',numRows,fieldCol).Value = val;
            else
                % Create new column for new field
                numCols = numCols + 1;
                newHeader = Sheet.get('Cells',1,numCols);
                newHeader.Select;
                newHeader.Value = field;
                newHeader.DisplayFormat.Font.FontStyle = 'Bold';
                % Write new value in new row
                Sheet.get('Cells',numRows,numCols).Value = val;
            end
        end
        
        % Autofit columns
        Sheet.UsedRange.Columns.AutoFit;
        
        % Create/modify chart
        if Sheet.ChartObjects.Count == 0
            % If no chart exists, create one
            chartShape = Sheet.Shapes.AddChart;
            chartShape.Select;
            Workbook.ActiveChart.ChartType = 'xlXYScatterLines';
            Workbook.ActiveChart.Axes(1).TickLabels.Orientation = 35;
        else
            % Select existing chart
            chartShape = Sheet.ChartObjects.Item(1);
            chartShape.Select;
        end
        % Set/update chart data
        range = Sheet.get('Range',Sheet.get('Cells',1,1),Sheet.get('Cells',numRows,numCols));
        Workbook.ActiveChart.SetSourceData(range)
        Workbook.ActiveChart.PlotBy = 'xlColumns';
        % Set/update chart position
        chartShape.Top = Sheet.get('Cells',numRows+2,1).Top;
        chartShape.Left = Sheet.get('Cells',numRows+2,1).Left+10;
        
        % Save the workbook
        invoke(Workbook, 'Save');
        msgbox('Export successful.');
    else
        % Don't have write access, file may be open in another program
        errordlg('Cannot export to excel file. The file may be open in another application.',...
            'Error');
    end
    % Close Excel
    invoke(Excel, 'Quit');
catch exception
    disp(getReport(exception));
    % Make sure to close excel if error occurs
    invoke(Excel, 'Quit');
end


% --- Executes on button press in grayscale_button_export.
function grayscale_button_export_Callback(hObject, eventdata, handles)
% hObject    handle to grayscale_button_export (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
rowHeaders = get(handles.grayscale_table,'RowName');
colHeaders = get(handles.grayscale_table,'ColumnName');
colHeaders = [colHeaders; 'Image'];
tableData = get(handles.grayscale_table,'Data');
exportDate = datestr(clock,'yyyymmdd_HHMMSS');
exportFolder = fullfile(pwd,'Log\Images',date);
if ~exist(exportFolder,'dir')
    mkdir(exportFolder);
end

try
    % Get handle to Excel COM Server
    Excel = actxserver('Excel.Application');
    Excel.DisplayAlerts = 0;
    % Open Workbook
    Workbooks = Excel.Workbooks;
    Workbook = Open(Workbooks,fullfile(pwd,'Log/','ProstateBrachyQA Log.xlsx'));
    
    if Workbook.ReadOnly == 0
        % Have write access to excel file
        % Get a handle to Sheets
        Sheets = Excel.ActiveWorkBook.Sheets;
        Sheet = [];
        for sheetNum = 1:Sheets.Count
            % Get existing Grayscale sheet
            if strcmp(Sheets.Item(sheetNum).Name,'Grayscale')
                Sheet = get(Sheets, 'Item', sheetNum);
                break
            end
        end
        if isempty(Sheet)
            % Create new sheet after 1st sheet
            Sheet = Sheets.Add([],Sheets.Item(1));
        end
        Sheet.Name = 'Grayscale';
        Sheet.Activate;
        % Get number of last used row
        lastRow = Sheet.get('Cells').Find('*',Sheet.get('Cells',1,1),[],[],1,2);
        if ~isempty(lastRow)
            numRows = lastRow.Row;
        else
            % No data, initialize headers
            % Put test title in first cell
            Sheet.get('Cells',1,1).Value = 'Gradient Length';
            % Colour first cell yellow
            Sheet.get('Cells',1,1).Interior.ColorIndex = 6;
            headers = Sheet.get('Range',Sheet.get('Cells',1,2),Sheet.get('Cells',1,numel(colHeaders)+1));
            headers.Value = colHeaders';
            numRows = 1;
            % Set first row to bold
            Sheet.Range('1:1').Font.Bold = 1;
            % Freeze first row
            Sheet.Application.ActiveWindow.SplitRow = 1;
            Sheet.Application.ActiveWindow.FreezePanes = true;
            % Set first column to bold
            Sheet.Range('A:A').Font.Bold = 1;
        end
        numCols = Sheet.get('Cells').Find('*',Sheet.get('Cells',1,1),[],[],2,2).Column;
        xlData = Sheet.UsedRange.Value(1:numRows,1:numCols);
        
        % Write new data
        numRows = numRows + 1;
        % Write date
        dateCell = Sheet.get('Cells',numRows,1);
        dateCell.Value = date;
        % Write fields
        for n = 1:numel(colHeaders)-1
            field = colHeaders{n};
            val = tableData{n};
            % Remove any html formatting
            val = regexprep(val, '<.*?>','');
            [~,fieldCol] = find(strcmp(xlData(1,:),field));
            if ~isempty(fieldCol)
                % Column exists, write the value in new row
                Sheet.get('Cells',numRows,fieldCol).Value = val;
            else
                % Create new column for new field
                numCols = numCols + 1;
                newHeader = Sheet.get('Cells',1,numCols);
                newHeader.Select;
                newHeader.Value = field;
                newHeader.DisplayFormat.Font.FontStyle = 'Bold';
                % Write new value in new row
                Sheet.get('Cells',numRows,numCols).Value = val;
            end
        end
        % Save image and add to excel sheet
        im = handles.exportImages{handles.testNum}{1};
        if ~exist(fullfile(pwd,'Log\Images'),'dir')
            mkdir(fullfile(pwd,'Log\Images'));
        end
        filename = fullfile(exportFolder,[exportDate '_Grayscale.bmp']);
        imwrite(im,filename);
        [~,imageCol] = find(strcmp(xlData(1,:),'Image'),1);
        imageCell = Sheet.get('Cells',dateCell.Row,imageCol);
        % Add link to image
        Sheet.Hyperlinks.Add(imageCell,filename,[],[],'View Image');
        % Create comment for image preview on mouse hover
        imageCell.AddComment;
        imageCell.Comment.Shape.Fill.UserPicture(filename);
        imageCell.Comment.Shape.Width = size(im,2)/2;
        imageCell.Comment.Shape.Height = size(im,1)/2;
        
        % Autofit columns
        Sheet.UsedRange.Columns.AutoFit;
        
        % Create/modify chart
        if Sheet.ChartObjects.Count == 0
            % If no chart exists, create one
            chartShape = Sheet.Shapes.AddChart;
            chartShape.Select;
            Workbook.ActiveChart.ChartType = 'xlXYScatterLines';
            Workbook.ActiveChart.Axes(1).TickLabels.Orientation = 35;
        else
            % Select existing chart
            chartShape = Sheet.ChartObjects.Item(1);
            chartShape.Select;
        end
        % Set/update chart data
        rangeDate = Sheet.get('Range',Sheet.get('Cells',1,1),Sheet.get('Cells',numRows,1));
        rangeVal = Sheet.get('Range',Sheet.get('Cells',1,3),Sheet.get('Cells',numRows,3));
        range = Excel.Union(rangeDate,rangeVal);
        Workbook.ActiveChart.SetSourceData(range)
        Workbook.ActiveChart.PlotBy = 'xlColumns';
        Workbook.ActiveChart.HasTitle = 1;
        Workbook.ActiveChart.ChartTitle.Text = 'Gradient Length';
        % Set/update chart position
        chartShape.Top = Sheet.get('Cells',numRows+2,1).Top;
        chartShape.Left = Sheet.get('Cells',numRows+2,1).Left+10;
        
        % Save the workbook
        invoke(Workbook, 'Save');
        msgbox('Export successful.');
    else
        % Don't have write access, file may be open in another program
        errordlg('Cannot export to excel file. The file may be open in another application.',...
            'Error');
    end
    % Close Excel
    invoke(Excel, 'Quit');
catch exception
    disp(getReport(exception));
    % Make sure to close excel if error occurs
    invoke(Excel, 'Quit');
end


% --- Executes on button press in depth_button_export.
function depth_button_export_Callback(hObject, eventdata, handles)
% hObject    handle to depth_button_export (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
rowHeaders = get(handles.depth_table_axial,'RowName');
colHeaders = get(handles.depth_table_axial,'ColumnName');
colHeaders = [colHeaders; 'Image'];
tableDataAxial = get(handles.depth_table_axial,'Data');
tableDataLong = get(handles.depth_table_long,'Data');
freqAxial = get(handles.depth_text_freqAxial,'String');
freqLong = get(handles.depth_text_freqLong,'String');
exportDate = datestr(clock,'yyyymmdd_HHMMSS');
exportFolder = fullfile(pwd,'Log\Images',date);
if ~exist(exportFolder,'dir')
    mkdir(exportFolder);
end

try
    % Get handle to Excel COM Server
    Excel = actxserver('Excel.Application');
    Excel.DisplayAlerts = 0;
    % Open Workbook
    Workbooks = Excel.Workbooks;
    Workbook = Open(Workbooks,fullfile(pwd,'Log/','ProstateBrachyQA Log.xlsx'));
    
    if Workbook.ReadOnly == 0
        % Have write access to excel file
        % Get a handle to Sheets
        Sheets = Excel.ActiveWorkBook.Sheets;
        Sheet = [];
        for sheetNum = 1:Sheets.Count
            % Get existing Depth sheet
            if strcmp(Sheets.Item(sheetNum).Name,'Depth')
                Sheet = get(Sheets, 'Item', sheetNum);
                break
            end
        end
        if isempty(Sheet)
            % Create new sheet after 2nd sheet or last sheet
            if Sheets.Count >= 2
                Sheet = Sheets.Add([],Sheets.Item(2));
            else
                Sheet = Sheets.Add([],Sheets.Item(Sheets.Count));
            end
        end
        Sheet.Name = 'Depth';
        Sheet.Activate;
        % Get number of last used row
        lastRow = Sheet.get('Cells').Find('*',Sheet.get('Cells',1,1),[],[],1,2);
        if ~isempty(lastRow)
            numRows = lastRow.Row;
        else
            % No data, initialize headers
            % Put test title in first cell
            Sheet.get('Cells',1,1).Value = 'Depth';
            % Colour first cell yellow
            Sheet.get('Cells',1,1).Interior.ColorIndex = 6;
            % Axial plane headers
            titleAxial = Sheet.get('Range',Sheet.get('Cells',1,2),Sheet.get('Cells',1,numel(colHeaders)+2));
            titleAxial.MergeCells = 1;
            titleAxial.Value = 'Axial Plane';
            freqHeaderAxial = Sheet.get('Cells',2,2);
            freqHeaderAxial.Value = 'Freq (MHz)';
            headersAxial = Sheet.get('Range',Sheet.get('Cells',2,3),Sheet.get('Cells',2,numel(colHeaders)+2));
            headersAxial.Value = colHeaders';
            % Longitudinal plane headers
            longCol = headersAxial.End('xlToRight').Column + 1;
            titleLong = Sheet.get('Range',Sheet.get('Cells',1,longCol),Sheet.get('Cells',1,longCol+numel(colHeaders)));
            titleLong.MergeCells = 1;
            titleLong.Value = 'Longitudinal Plane';
            freqHeaderLong = Sheet.get('Cells',2,longCol);
            freqHeaderLong.Value = 'Freq (MHz)';
            headersAxial = Sheet.get('Range',Sheet.get('Cells',2,longCol+1),Sheet.get('Cells',2,longCol+numel(colHeaders)));
            headersAxial.Value = colHeaders';
            numRows = 2;
            % Set first 2 rows to bold
            Sheet.Range('1:2').Font.Bold = 1;
            % Freeze first 2 rows
            Sheet.Application.ActiveWindow.SplitRow = 2;
            Sheet.Application.ActiveWindow.FreezePanes = true;
            % Set first column to bold
            Sheet.Range('A:A').Font.Bold = 1;
        end
        numCols = Sheet.get('Cells').Find('*',Sheet.get('Cells',1,1),[],[],2,2).Column;
        xlData = Sheet.UsedRange.Value(1:numRows,1:numCols);
        
        % Write new data
        numRows = numRows + 1;
        % Write date
        dateCell = Sheet.get('Cells',numRows,1);
        dateCell.Value = date;
        
        % Axial plane
        % Write frequency
        [~,freqCol] = find(strcmp(xlData(2,:),'Freq (MHz)'),1);
        freqCell = Sheet.get('Cells',numRows,freqCol);
        freqCell.Value = freqAxial;
        % Write fields 
        for n = 1:numel(colHeaders)-1
            field = colHeaders{n};
            val = tableDataAxial{n};
            % Remove any html formatting
            val = regexprep(val, '<.*?>','');
            [~,fieldCol] = find(strcmp(xlData(2,:),field),1);
            if ~isempty(fieldCol)
                % Column exists, write the value in new row
                Sheet.get('Cells',numRows,fieldCol).Value = val;
            end
        end
        % Save axial image and add to excel sheet
        imAxial = handles.exportImages{handles.testNum}{1};
        if ~exist(fullfile(pwd,'Log\Images'),'dir')
            mkdir(fullfile(pwd,'Log\Images'));
        end
        filename = fullfile(exportFolder,[exportDate '_Depth_Axial.bmp']);
        imwrite(imAxial,filename);
        [~,imageCol] = find(strcmp(xlData(2,:),'Image'),1);
        imageCell = Sheet.get('Cells',dateCell.Row,imageCol);
        % Add link to image
        Sheet.Hyperlinks.Add(imageCell,filename,[],[],'View Image');
        % Create comment for image preview on mouse hover
        imageCell.AddComment;
        imageCell.Comment.Shape.Fill.UserPicture(filename);
        imageCell.Comment.Shape.Width = size(imAxial,2)/2;
        imageCell.Comment.Shape.Height = size(imAxial,1)/2;
        
        % Longitudinal plane
        % Write frequency
        [~,freqCol] = find(strcmp(xlData(2,:),'Freq (MHz)'),1,'last');
        freqCell = Sheet.get('Cells',numRows,freqCol);
        freqCell.Value = freqLong;
        for n = 1:numel(colHeaders)-1
            field = colHeaders{n};
            val = tableDataLong{n};
            % Remove any html formatting
            val = regexprep(val, '<.*?>','');
            [~,fieldCol] = find(strcmp(xlData(2,:),field),1,'last');
            if ~isempty(fieldCol)
                % Column exists, write the value in new row
                Sheet.get('Cells',numRows,fieldCol).Value = val;
            end
        end
        % Save longitudinal image and add to excel sheet
        imLong = handles.exportImages{handles.testNum}{2};
        if ~exist(fullfile(pwd,'Log\Images'),'dir')
            mkdir(fullfile(pwd,'Log\Images'));
        end
        filename = fullfile(exportFolder,[exportDate '_Depth_Longitudinal.bmp']);
        imwrite(imLong,filename);
        [~,imageCol] = find(strcmp(xlData(2,:),'Image'),1,'last');
        imageCell = Sheet.get('Cells',dateCell.Row,imageCol);
        % Add link to image
        Sheet.Hyperlinks.Add(imageCell,filename,[],[],'View Image');
        % Create comment for image preview on mouse hover
        imageCell.AddComment;
        imageCell.Comment.Shape.Fill.UserPicture(filename);
        imageCell.Comment.Shape.Width = size(imLong,2)/2;
        imageCell.Comment.Shape.Height = size(imLong,1)/2;
        
        % Autofit columns
        Sheet.UsedRange.Columns.AutoFit;
        
        % Delete existing charts
        if Sheet.ChartObjects.Count > 0
            Sheet.ChartObjects.Delete;
        end
        
        % Axial chart
        axialCol = 2;
        freqColumn = Sheet.get('Range',Sheet.get('Cells',3,2),Sheet.get('Cells',dateCell.Row,2));
        if numel(freqColumn.Value) == 1
            axialFreqs = freqColumn.Value;
        else
            axialFreqs = unique([freqColumn.Value{:}]);
        end
        axialFreqs = axialFreqs(~isnan(axialFreqs));
        chartShape = cell(1,numel(axialFreqs));
        % Add separate chart for each frequency
        for f = 1:numel(axialFreqs)
            freq = axialFreqs(f);
            % Create chart
            chartShape{f} = Sheet.Shapes.AddChart;
            chartShape{f}.Select;
            Workbook.ActiveChart.ChartType = 'xlXYScatterLines';
            Workbook.ActiveChart.Axes(1).TickLabels.Orientation = 35;
            % Clear default data
            Workbook.ActiveChart.ChartArea.ClearContents;
            % Get first row for this frequency
            if iscell(freqColumn.Value)
                [~,firstRow] = find([freqColumn.Value{:}]==freq,1);
            else
                [~,firstRow] = find([freqColumn.Value]==freq,1);
            end
            % Get chart shape
            chartShape{f} = Sheet.ChartObjects.Item(f);
            chartShape{f}.Select;
            % Set/update chart data
            % Create series
            seriesCollection = Workbook.ActiveChart.SeriesCollection;
            series = seriesCollection.NewSeries;
            series.Name = 'Depth (mm)';
            % X Data
            dateColumn = Sheet.get('Range',Sheet.get('Cells',firstRow+2,1),Sheet.get('Cells',dateCell.Row,1));
            dateRange = dateColumn.Cells.Item(1);
            for n = firstRow+1:dateColumn.Cells.Count;
                % Only add data for current frequency
                if freqColumn.Cells.Item(n).Value == freq
                    dateRange = Excel.Union(dateRange,dateColumn.Cells.Item(n));
                end
            end
            series.XValues = dateRange;
            % Y Data
            valColumn = Sheet.get('Range',Sheet.get('Cells',firstRow+2,axialCol+2),...
                Sheet.get('Cells',dateCell.Row,axialCol+2));
            valRange = valColumn.Cells.Item(1);
            for n = firstRow+1:valColumn.Cells.Count;
                % Only add data for current frequency
                if freqColumn.Cells.Item(n).Value == freq
                    valRange = Excel.Union(valRange,valColumn.Cells.Item(n));
                end
            end
            series.Values = valRange;
            Workbook.ActiveChart.HasTitle = 1;
            Workbook.ActiveChart.ChartTitle.Text = [num2str(freq) ' MHz Depth (Axial)'];
            % Set/update chart position
            if f == 1
                chartCell = Sheet.get('Cells',numRows+2,1);
            else
                chartCell = Sheet.get('Cells',chartShape{f-1}.BottomRightCell.Row+1,1);
            end
            chartShape{f}.Top = chartCell.Top;
            chartShape{f}.Left = chartCell.Left+10;
        end
        
        % Longitudinal chart
        longCol = numel(colHeaders)+3;
        freqColumn = Sheet.get('Range',Sheet.get('Cells',3,longCol),Sheet.get('Cells',dateCell.Row,longCol));
        if numel(freqColumn.Value) == 1
            longFreqs = freqColumn.Value;
        else
            longFreqs = unique([freqColumn.Value{:}]);
        end
        longFreqs = longFreqs(~isnan(longFreqs));
        % Add separate chart for each frequency
        for g = 1:numel(longFreqs)
            freq = longFreqs(g);
            chartNum = numel(axialFreqs)+g;
            % Create chart
            chartShape{chartNum} = Sheet.Shapes.AddChart;
            chartShape{chartNum}.Select;
            Workbook.ActiveChart.ChartType = 'xlXYScatterLines';
            Workbook.ActiveChart.Axes(1).TickLabels.Orientation = 35;
            % Clear default data
            Workbook.ActiveChart.ChartArea.ClearContents;
            % Get first row for this frequency
            if iscell(freqColumn.Value)
                [~,firstRow] = find([freqColumn.Value{:}]==freq,1);
            else
                [~,firstRow] = find([freqColumn.Value]==freq,1);
            end
            % Get chart shape
            chartShape{chartNum} = Sheet.ChartObjects.Item(chartNum);
            chartShape{chartNum}.Select;
            % Set/update chart data
            % Create series
            seriesCollection = Workbook.ActiveChart.SeriesCollection;
            series = seriesCollection.NewSeries;
            series.Name = 'Depth (mm)';
            % X Data
            dateColumn = Sheet.get('Range',Sheet.get('Cells',firstRow+2,1),Sheet.get('Cells',dateCell.Row,1));
            dateRange = dateColumn.Cells.Item(1);
            for n = firstRow+1:dateColumn.Cells.Count;
                % Only add data for current frequency
                if freqColumn.Cells.Item(n).Value == freq
                    dateRange = Excel.Union(dateRange,dateColumn.Cells.Item(n));
                end
            end
            series.XValues = dateRange;
            % Y Data
            valColumn = Sheet.get('Range',Sheet.get('Cells',firstRow+2,longCol+2),...
                Sheet.get('Cells',dateCell.Row,longCol+2));
            valRange = valColumn.Cells.Item(1);
            for n = firstRow+1:valColumn.Cells.Count;
                % Only add data for current frequency
                if freqColumn.Cells.Item(n).Value == freq
                    valRange = Excel.Union(valRange,valColumn.Cells.Item(n));
                end
            end
            series.Values = valRange;
            Workbook.ActiveChart.HasTitle = 1;
            Workbook.ActiveChart.ChartTitle.Text = [num2str(freq) ' MHz Depth (Longitudinal)'];
            % Set/update chart position
            if g == 1
                chartCell = Sheet.get('Cells',numRows+2,chartShape{1}.BottomRightCell.Column+1);
            else
                chartCell = Sheet.get('Cells',chartShape{chartNum-1}.BottomRightCell.Row+1,chartShape{1}.BottomRightCell.Column+1);
            end
            chartShape{chartNum}.Top = chartCell.Top;
            chartShape{chartNum}.Left = chartCell.Left+10;
        end
        
        % Save the workbook
        invoke(Workbook, 'Save');
        msgbox('Export successful.');
    else
        % Don't have write access, file may be open in another program
        errordlg('Cannot export to excel file. The file may be open in another application.',...
            'Error');
    end
    % Close Excel
    invoke(Excel, 'Quit');
catch exception
    disp(getReport(exception));
    % Make sure to close excel if error occurs
    invoke(Excel, 'Quit');
end


% --- Executes on button press in axialResolution_button_export.
function axialResolution_button_export_Callback(hObject, eventdata, handles)
% hObject    handle to axialResolution_button_export (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
rowHeadersAxial = get(handles.axialResolution_table_axial,'RowName');
rowHeadersLong = get(handles.axialResolution_table_long,'RowName');
colHeaders = get(handles.axialResolution_table_axial,'ColumnName');
colHeaders = [colHeaders; 'Image'];
tableDataAxial = get(handles.axialResolution_table_axial,'Data');
tableDataLong = get(handles.axialResolution_table_long,'Data');
freqAxial = get(handles.axialResolution_text_freqAxial,'String');
freqLong = get(handles.axialResolution_text_freqLong,'String');
exportDate = datestr(clock,'yyyymmdd_HHMMSS');
exportFolder = fullfile(pwd,'Log\Images',date);
if ~exist(exportFolder,'dir')
    mkdir(exportFolder);
end

try
    % Get handle to Excel COM Server
    Excel = actxserver('Excel.Application');
    Excel.DisplayAlerts = 0;
    % Open Workbook
    Workbooks = Excel.Workbooks;
    Workbook = Open(Workbooks,fullfile(pwd,'Log/','ProstateBrachyQA Log.xlsx'));
    
    if Workbook.ReadOnly == 0
        % Have write access to excel file
        % Get a handle to Sheets
        Sheets = Excel.ActiveWorkBook.Sheets;
        Sheet = [];
        for sheetNum = 1:Sheets.Count
            % Get existing Axial Resolution sheet
            if strcmp(Sheets.Item(sheetNum).Name,'Axial Resolution')
                Sheet = get(Sheets, 'Item', sheetNum);
                break
            end
        end
        if isempty(Sheet)
            % Create new sheet after 3rd sheet or last sheet
            if Sheets.Count >= 3
                Sheet = Sheets.Add([],Sheets.Item(3));
            else
                Sheet = Sheets.Add([],Sheets.Item(Sheets.Count));
            end
        end
        Sheet.Name = 'Axial Resolution';
        Sheet.Activate;
        % Get number of last used row
        lastRow = Sheet.get('Cells').Find('*',Sheet.get('Cells',1,1),[],[],1,2);
        if ~isempty(lastRow)
            numRows = lastRow.Row;
        else
            % No data, initialize headers
            % Put test title in first cell
            Sheet.get('Cells',1,1).Value = 'Axial Resolution';
            % Colour first cell yellow
            Sheet.get('Cells',1,1).Interior.ColorIndex = 6;
            % Axial plane headers
            titleAxial = Sheet.get('Range',Sheet.get('Cells',1,2),Sheet.get('Cells',1,numel(colHeaders)+2));
            titleAxial.MergeCells = 1;
            titleAxial.Value = 'Axial Plane';
            freqHeaderAxial = Sheet.get('Cells',2,2);
            freqHeaderAxial.Value = 'Freq (MHz)';
            headersAxial = Sheet.get('Range',Sheet.get('Cells',2,4),Sheet.get('Cells',2,numel(colHeaders)+3));
            headersAxial.Value = colHeaders';
            % Longitudinal plane headers
            longCol = headersAxial.End('xlToRight').Column + 1;
            titleLong = Sheet.get('Range',Sheet.get('Cells',1,longCol),Sheet.get('Cells',1,longCol+numel(colHeaders)));
            titleLong.MergeCells = 1;
            titleLong.Value = 'Longitudinal Plane';
            freqHeaderLong = Sheet.get('Cells',2,longCol);
            freqHeaderLong.Value = 'Freq (MHz)';
            headersLong = Sheet.get('Range',Sheet.get('Cells',2,longCol+2),Sheet.get('Cells',2,longCol+numel(colHeaders)+1));
            headersLong.Value = colHeaders';
            numRows = 2;
            % Set first 2 rows to bold
            Sheet.Range('1:2').Font.Bold = 1;
            % Freeze first 2 rows
            Sheet.Application.ActiveWindow.SplitRow = 2;
            Sheet.Application.ActiveWindow.FreezePanes = true;
            % Set first column to bold
            Sheet.Range('A:A').Font.Bold = 1;
        end
        numCols = Sheet.get('Cells').Find('*',Sheet.get('Cells',1,1),[],[],2,2).Column;
        xlData = Sheet.UsedRange.Value(1:numRows,1:numCols);
        
        % Write new data
        if numRows > 2
            % Add extra space between dates
            numRows = numRows + 2;
        else
            % First entry, no need to add extra space
            numRows = numRows + 1;
        end
        % Write date
        dateCell = Sheet.get('Cells',numRows,1);
        dateCell.Value = date;
        
        % Axial plane
        % Write frequency
        [~,freqCol] = find(strcmp(xlData(2,:),'Freq (MHz)'),1);
        freqCell = Sheet.get('Cells',dateCell.Row,freqCol);
        freqCell.Value = freqAxial;
        % Write row headers
        rowHeaderRange = Sheet.get('Range',Sheet.get('Cells',dateCell.Row,freqCell.Column+1),Sheet.get('Cells',dateCell.Row+numel(rowHeadersAxial)-1,freqCell.Column+1));
        rowHeaderRange.Value = rowHeadersAxial;
        rowHeaderRange.Font.Bold = 1;
        % Write values
        for m = 1:numel(rowHeadersAxial)
            rowNum = dateCell.Row + m - 1;
            for n = 1:numel(colHeaders)-1
                field = colHeaders{n};
                val = tableDataAxial{m,n};
                % Remove any html formatting
                val = regexprep(val, '<.*?>','');
                [~,fieldCol] = find(strcmp(xlData(2,:),field),1);
                if ~isempty(fieldCol)
                    % Column exists, write the value in new row
                    Sheet.get('Cells',rowNum,fieldCol).Value = val;
                end
            end
        end
        % Save axial image and add to excel sheet
        imAxial = handles.exportImages{handles.testNum}{1};
        if ~exist(fullfile(pwd,'Log\Images'),'dir')
            mkdir(fullfile(pwd,'Log\Images'));
        end
        filename = fullfile(exportFolder,[exportDate '_AxialResolution_Axial.bmp']);
        imwrite(imAxial,filename);
        [~,imageCol] = find(strcmp(xlData(2,:),'Image'),1);
        imageCell = Sheet.get('Cells',dateCell.Row,imageCol);
        % Add link to image
        Sheet.Hyperlinks.Add(imageCell,filename,[],[],'View Image');
        % Create comment for image preview on mouse hover
        imageCell.AddComment;
        imageCell.Comment.Shape.Fill.UserPicture(filename);
        imageCell.Comment.Shape.Width = size(imAxial,2)/2;
        imageCell.Comment.Shape.Height = size(imAxial,1)/2;
        
        % Longitudinal plane
        longCol = numel(colHeaders)+4;
        % Write frequency
        [~,freqCol] = find(strcmp(xlData(2,:),'Freq (MHz)'),1,'last');
        freqCell = Sheet.get('Cells',dateCell.Row,freqCol);
        freqCell.Value = freqLong;
        % Write row headers
        rowHeaderRange = Sheet.get('Range',Sheet.get('Cells',dateCell.Row,longCol+1),Sheet.get('Cells',dateCell.Row+numel(rowHeadersLong)-1,longCol+1));
        rowHeaderRange.Value = rowHeadersLong;
        rowHeaderRange.Font.Bold = 1;
        % Write values
        for m = 1:numel(rowHeadersLong)
            rowNum = dateCell.Row + m - 1;
            for n = 1:numel(colHeaders)-1
                field = colHeaders{n};
                val = tableDataLong{m,n};
                % Remove any html formatting
                val = regexprep(val, '<.*?>','');
                [~,fieldCol] = find(strcmp(xlData(2,:),field),1,'last');
                if ~isempty(fieldCol)
                    % Column exists, write the value in new row
                    Sheet.get('Cells',rowNum,fieldCol).Value = val;
                end
            end
        end
        % Save longitudinal image and add to excel sheet
        imLong = handles.exportImages{handles.testNum}{2};
        if ~exist(fullfile(pwd,'Log\Images'),'dir')
            mkdir(fullfile(pwd,'Log\Images'));
        end
        filename = fullfile(exportFolder,[exportDate '_AxialResolution_Longitudinal.bmp']);
        imwrite(imLong,filename);
        [~,imageCol] = find(strcmp(xlData(2,:),'Image'),1,'last');
        imageCell = Sheet.get('Cells',dateCell.Row,imageCol);
        % Add link to image
        Sheet.Hyperlinks.Add(imageCell,filename,[],[],'View Image');
        % Create comment for image preview on mouse hover
        imageCell.AddComment;
        imageCell.Comment.Shape.Fill.UserPicture(filename);
        imageCell.Comment.Shape.Width = size(imLong,2)/2;
        imageCell.Comment.Shape.Height = size(imLong,1)/2;
        
        % Update number of rows
        numRows = Sheet.get('Cells').Find('*',Sheet.get('Cells',1,1),[],[],1,2).Row;
        
        % Autofit columns
        Sheet.UsedRange.Columns.AutoFit;
        
        % Delete existing charts
        if Sheet.ChartObjects.Count > 0
            Sheet.ChartObjects.Delete;
        end
        
        % Number of rows between measurements of different dates
        interval = 5;
        
        % Axial charts
        axialCol = 3;
        freqColumn = Sheet.get('Range',Sheet.get('Cells',3,2),Sheet.get('Cells',dateCell.Row,2));
        if numel(freqColumn.Value) == 1
            axialFreqs = freqColumn.Value;
        else
            axialFreqs = unique([freqColumn.Value{:}]);
        end
        axialFreqs = axialFreqs(~isnan(axialFreqs));
        chartShape = cell(1,numel(axialFreqs));
        % Add separate chart for each frequency
        for f = 1:numel(axialFreqs)
            freq = axialFreqs(f);
            % Create chart
            chartShape{f} = Sheet.Shapes.AddChart;
            chartShape{f}.Select;
            Workbook.ActiveChart.ChartType = 'xlXYScatterLines';
            Workbook.ActiveChart.Axes(1).TickLabels.Orientation = 35;
            % Clear default data
            Workbook.ActiveChart.ChartArea.ClearContents;
            % Get first row for this frequency
            if iscell(freqColumn.Value)
                [~,firstRow] = find([freqColumn.Value{:}]==freq,1);
            else
                [~,firstRow] = find([freqColumn.Value]==freq,1);
            end
            % Get chart shape
            chartShape{f} = Sheet.ChartObjects.Item(f);
            chartShape{f}.Select;
            % Set/update chart data
            seriesCollection = Workbook.ActiveChart.SeriesCollection;
            if seriesCollection.Count == 0
                % Create series
                for s = 1:numel(rowHeadersAxial)
                    seriesCollection.NewSeries;
                end
            end
            for s = 1:numel(rowHeadersAxial)
                series = seriesCollection.Item(s);
                series.Name = rowHeadersAxial{s};
                % X Data
                dateColumn = Sheet.get('Range',Sheet.get('Cells',firstRow+2,1),Sheet.get('Cells',dateCell.Row,1));
                dateRange = dateColumn.Cells.Item(1);
                for n = firstRow+interval:interval:dateColumn.Cells.Count;
                    % Only add data for current frequency
                    if freqColumn.Cells.Item(n).Value == freq
                        dateRange = Excel.Union(dateRange,dateColumn.Cells.Item(n));
                    end
                end
                series.XValues = dateRange;
                % Y Data
                valColumn = Sheet.get('Range',Sheet.get('Cells',firstRow+2,axialCol+2),...
                    Sheet.get('Cells',dateCell.Row+numel(rowHeadersAxial)-1,axialCol+2));
                valRange = valColumn.Cells.Item(s);
                for n = s+interval:interval:valColumn.Cells.Count;
                    % Only add data for current frequency
                    if freqColumn.Cells.Item(n-s+1).Value == freq
                        valRange = Excel.Union(valRange,valColumn.Cells.Item(n));
                    end
                end
                series.Values = valRange;
            end
            Workbook.ActiveChart.HasTitle = 1;
            Workbook.ActiveChart.ChartTitle.Text = [num2str(freq) ' MHz Axial Resolution (Axial)'];
            % Set/update chart position
            if f == 1
                chartCell = Sheet.get('Cells',numRows+2,1);
            else
                chartCell = Sheet.get('Cells',chartShape{f-1}.BottomRightCell.Row+1,1);
            end
            chartShape{f}.Top = chartCell.Top;
            chartShape{f}.Left = chartCell.Left+10;
        end
        
        % Longitudinal chart
        longCol = numel(colHeaders)+4;
        freqColumn = Sheet.get('Range',Sheet.get('Cells',3,longCol),Sheet.get('Cells',dateCell.Row,longCol));
        if numel(freqColumn.Value) == 1
            longFreqs = freqColumn.Value;
        else
            longFreqs = unique([freqColumn.Value{:}]);
        end
        longFreqs = longFreqs(~isnan(longFreqs));
        % Add separate chart for each frequency
        for g = 1:numel(longFreqs)
            freq = longFreqs(g);
            chartNum = numel(axialFreqs)+g;
            % Create chart
            chartShape{chartNum} = Sheet.Shapes.AddChart;
            chartShape{chartNum}.Select;
            Workbook.ActiveChart.ChartType = 'xlXYScatterLines';
            Workbook.ActiveChart.Axes(1).TickLabels.Orientation = 35;
            % Clear default data
            Workbook.ActiveChart.ChartArea.ClearContents;
            % Get first row for this frequency
            if iscell(freqColumn.Value)
                [~,firstRow] = find([freqColumn.Value{:}]==freq,1);
            else
                [~,firstRow] = find([freqColumn.Value]==freq,1);
            end
            % Get chart shape
            chartShape{chartNum} = Sheet.ChartObjects.Item(chartNum);
            chartShape{chartNum}.Select;
            % Set/update chart data
            seriesCollection = Workbook.ActiveChart.SeriesCollection;
            if seriesCollection.Count == 0
                % Create series
                for s = 1:numel(rowHeadersLong)
                    seriesCollection.NewSeries;
                end
            end
            for s = 1:numel(rowHeadersLong)
                series = seriesCollection.Item(s);
                series.Name = rowHeadersLong{s};
                % X Data
                dateColumn = Sheet.get('Range',Sheet.get('Cells',firstRow+2,1),Sheet.get('Cells',dateCell.Row,1));
                dateRange = dateColumn.Cells.Item(1);
                for n = firstRow+interval:interval:dateColumn.Cells.Count;
                    % Only add data for current frequency
                    if freqColumn.Cells.Item(n).Value == freq
                        dateRange = Excel.Union(dateRange,dateColumn.Cells.Item(n));
                    end
                end
                series.XValues = dateRange;
                % Y Data
                valColumn = Sheet.get('Range',Sheet.get('Cells',firstRow+2,longCol+3),...
                    Sheet.get('Cells',dateCell.Row+numel(rowHeadersLong)-1,longCol+3));
                valRange = valColumn.Cells.Item(s);
                for n = s+interval:interval:valColumn.Cells.Count;
                    % Only add data for current frequency
                    if freqColumn.Cells.Item(n-s+1).Value == freq
                        valRange = Excel.Union(valRange,valColumn.Cells.Item(n));
                    end
                end
                series.Values = valRange;
            end
            Workbook.ActiveChart.HasTitle = 1;
            Workbook.ActiveChart.ChartTitle.Text = [num2str(freq) ' MHz Axial Resolution (Longitudinal)'];
            % Set/update chart position
            if g == 1
                chartCell = Sheet.get('Cells',numRows+2,chartShape{1}.BottomRightCell.Column+1);
            else
                chartCell = Sheet.get('Cells',chartShape{chartNum-1}.BottomRightCell.Row+1,chartShape{1}.BottomRightCell.Column+1);
            end
            chartShape{chartNum}.Top = chartCell.Top;
            chartShape{chartNum}.Left = chartCell.Left+10;
        end
        
        % Save the workbook
        invoke(Workbook, 'Save');
        msgbox('Export successful.');
    else
        % Don't have write access, file may be open in another program
        errordlg('Cannot export to excel file. The file may be open in another application.',...
            'Error');
    end
    % Close Excel
    invoke(Excel, 'Quit');
catch exception
    disp(getReport(exception));
    % Make sure to close excel if error occurs
    invoke(Excel, 'Quit');
end

% --- Executes on button press in lateralResolution_button_export.
function lateralResolution_button_export_Callback(hObject, eventdata, handles)
% hObject    handle to axialResolution_button_export (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
rowHeadersAxial = get(handles.lateralResolution_table_axial,'RowName');
rowHeadersLong = get(handles.lateralResolution_table_long,'RowName');
colHeaders = get(handles.lateralResolution_table_axial,'ColumnName');
colHeaders = [colHeaders; 'Image'];
tableDataAxial = get(handles.lateralResolution_table_axial,'Data');
tableDataLong = get(handles.lateralResolution_table_long,'Data');
exportDate = datestr(clock,'yyyymmdd_HHMMSS');
exportFolder = fullfile(pwd,'Log\Images',date);
if ~exist(exportFolder,'dir')
    mkdir(exportFolder);
end

try
    % Get handle to Excel COM Server
    Excel = actxserver('Excel.Application');
    Excel.DisplayAlerts = 0;
    % Open Workbook
    Workbooks = Excel.Workbooks;
    Workbook = Open(Workbooks,fullfile(pwd,'Log/','ProstateBrachyQA Log.xlsx'));
    
    if Workbook.ReadOnly == 0
        % Have write access to excel file
        % Get a handle to Sheets
        Sheets = Excel.ActiveWorkBook.Sheets;
        Sheet = [];
        for sheetNum = 1:Sheets.Count
            % Get existing Lateral Resolution sheet
            if strcmp(Sheets.Item(sheetNum).Name,'Lateral Resolution')
                Sheet = get(Sheets, 'Item', sheetNum);
                break
            end
        end
        if isempty(Sheet)
            % Create new sheet after 4th sheet or last sheet
            if Sheets.Count >= 4
                Sheet = Sheets.Add([],Sheets.Item(4));
            else
                Sheet = Sheets.Add([],Sheets.Item(Sheets.Count));
            end
        end
        Sheet.Name = 'Lateral Resolution';
        Sheet.Activate;
        % Get number of last used row
        lastRow = Sheet.get('Cells').Find('*',Sheet.get('Cells',1,1),[],[],1,2);
        if ~isempty(lastRow)
            numRows = lastRow.Row;
        else
            % No data, initialize headers
            % Put test title in first cell
            Sheet.get('Cells',1,1).Value = 'Lateral Resolution';
            % Colour first cell yellow
            Sheet.get('Cells',1,1).Interior.ColorIndex = 6;
            % Axial plane headers
            titleAxial = Sheet.get('Range',Sheet.get('Cells',1,2),Sheet.get('Cells',1,numel(colHeaders)+2));
            titleAxial.MergeCells = 1;
            titleAxial.Value = 'Axial Plane';
            headersAxial = Sheet.get('Range',Sheet.get('Cells',2,3),Sheet.get('Cells',2,numel(colHeaders)+2));
            headersAxial.Value = colHeaders';
            % Longitudinal plane headers
            longCol = headersAxial.End('xlToRight').Column + 1;
            titleLong = Sheet.get('Range',Sheet.get('Cells',1,longCol),Sheet.get('Cells',1,longCol+numel(colHeaders)-1));
            titleLong.MergeCells = 1;
            titleLong.Value = 'Longitudinal Plane';
            headersLong = Sheet.get('Range',Sheet.get('Cells',2,longCol+1),Sheet.get('Cells',2,longCol+numel(colHeaders)));
            headersLong.Value = colHeaders';
            numRows = 2;
            % Set first 2 rows to bold
            Sheet.Range('1:2').Font.Bold = 1;
            % Freeze first 2 rows
            Sheet.Application.ActiveWindow.SplitRow = 2;
            Sheet.Application.ActiveWindow.FreezePanes = true;
            % Set first column to bold
            Sheet.Range('A:A').Font.Bold = 1;
        end
        numCols = Sheet.get('Cells').Find('*',Sheet.get('Cells',1,1),[],[],2,2).Column;
        xlData = Sheet.UsedRange.Value(1:numRows,1:numCols);
        
        % Write new data
        if numRows > 2
            % Add extra space between dates
            numRows = numRows + 2;
        else
            % First entry, no need to add extra space
            numRows = numRows + 1;
        end
        % Write date
        dateCell = Sheet.get('Cells',numRows,1);
        dateCell.Value = date;
        
        % Axial plane
        % Write row headers
        rowHeaderRange = Sheet.get('Range',Sheet.get('Cells',dateCell.Row,2),Sheet.get('Cells',dateCell.Row+numel(rowHeadersAxial)-1,2));
        rowHeaderRange.Value = rowHeadersAxial;
        rowHeaderRange.Font.Bold = 1;
        % Write values
        for m = 1:numel(rowHeadersAxial)
            rowNum = dateCell.Row + m - 1;
            for n = 1:numel(colHeaders)-1
                field = colHeaders{n};
                val = tableDataAxial{m,n};
                % Remove any html formatting
                val = regexprep(val, '<.*?>','');
                [~,fieldCol] = find(strcmp(xlData(2,:),field),1);
                if ~isempty(fieldCol)
                    % Column exists, write the value in new row
                    Sheet.get('Cells',rowNum,fieldCol).Value = val;
                end
            end
        end
        % Save axial image and add to excel sheet
        imAxial = handles.exportImages{handles.testNum}{1};
        if ~exist(fullfile(pwd,'Log\Images'),'dir')
            mkdir(fullfile(pwd,'Log\Images'));
        end
        filename = fullfile(exportFolder,[exportDate '_LateralResolution_Axial.bmp']);
        imwrite(imAxial,filename);
        [~,imageCol] = find(strcmp(xlData(2,:),'Image'),1);
        imageCell = Sheet.get('Cells',dateCell.Row,imageCol);
        % Add link to image
        Sheet.Hyperlinks.Add(imageCell,filename,[],[],'View Image');
        % Create comment for image preview on mouse hover
        imageCell.AddComment;
        imageCell.Comment.Shape.Fill.UserPicture(filename);
        imageCell.Comment.Shape.Width = size(imAxial,2)/2;
        imageCell.Comment.Shape.Height = size(imAxial,1)/2;
        
        % Longitudinal plane
        longCol = numel(colHeaders)+3;
        % Write row headers
        rowHeaderRange = Sheet.get('Range',Sheet.get('Cells',dateCell.Row,longCol),Sheet.get('Cells',dateCell.Row+numel(rowHeadersLong)-1,longCol));
        rowHeaderRange.Value = rowHeadersLong;
        rowHeaderRange.Font.Bold = 1;
        % Write values
        for m = 1:numel(rowHeadersLong)
            rowNum = dateCell.Row + m - 1;
            for n = 1:numel(colHeaders)-1
                field = colHeaders{n};
                val = tableDataLong{m,n};
                % Remove any html formatting
                val = regexprep(val, '<.*?>','');
                [~,fieldCol] = find(strcmp(xlData(2,:),field),1,'last');
                if ~isempty(fieldCol)
                    % Column exists, write the value in new row
                    Sheet.get('Cells',rowNum,fieldCol).Value = val;
                end
            end
        end
        % Save longitudinal image and add to excel sheet
        im = handles.exportImages{handles.testNum}{2};
        if ~exist(fullfile(pwd,'Log\Images'),'dir')
            mkdir(fullfile(pwd,'Log\Images'));
        end
        filename = fullfile(exportFolder,[exportDate '_LateralResolution_Longitudinal.bmp']);
        imwrite(im,filename);
        [~,imageCol] = find(strcmp(xlData(2,:),'Image'),1,'last');
        imageCell = Sheet.get('Cells',dateCell.Row,imageCol);
        % Add link to image
        Sheet.Hyperlinks.Add(imageCell,filename,[],[],'View Image');
        % Create comment for image preview on mouse hover
        imageCell.AddComment;
        imageCell.Comment.Shape.Fill.UserPicture(filename);
        imageCell.Comment.Shape.Width = size(im,2)/2;
        imageCell.Comment.Shape.Height = size(im,1)/2;
        
        % Update number of rows
        numRows = Sheet.get('Cells').Find('*',Sheet.get('Cells',1,1),[],[],1,2).Row;
        
        % Autofit columns
        Sheet.UsedRange.Columns.AutoFit;
        
        % Create/modify chart
        if Sheet.ChartObjects.Count == 0
            % If no chart exists, create them
            for n = 1:2
                chartShape = Sheet.Shapes.AddChart;
                chartShape.Select;
                Workbook.ActiveChart.ChartType = 'xlXYScatterLines';
                Workbook.ActiveChart.Axes(1).TickLabels.Orientation = 35;
                % Clear default data
                Workbook.ActiveChart.ChartArea.ClearContents;
            end
        end
        
        % Number of rows between measurements of different dates
        interval = 5;
        
        % Axial chart
        axialCol = 2;
        chartShape1 = Sheet.ChartObjects.Item(1);
        chartShape1.Select;
        % Set/update chart data
        seriesCollection = Workbook.ActiveChart.SeriesCollection;
        if seriesCollection.Count == 0
            % Create series
            for s = 1:numel(rowHeadersAxial)
                seriesCollection.NewSeries;
            end
        end
        for s = 1:numel(rowHeadersAxial)
            series = seriesCollection.Item(s);
            series.Name = rowHeadersAxial{s};
            % X Data
            dateColumn = Sheet.get('Range',Sheet.get('Cells',3,1),Sheet.get('Cells',dateCell.Row,1));
            dateRange = dateColumn.Cells.Item(1);
            for n = 1+interval:interval:dateColumn.Cells.Count;
                dateRange = Excel.Union(dateRange,dateColumn.Cells.Item(n));
            end
            series.XValues = dateRange;
            % Y Data
            valColumn = Sheet.get('Range',Sheet.get('Cells',3,axialCol+2),...
                Sheet.get('Cells',dateCell.Row+numel(rowHeadersAxial)-1,axialCol+2));
            valRange = valColumn.Cells.Item(s);
            for n = s+interval:interval:valColumn.Cells.Count;
                valRange = Excel.Union(valRange,valColumn.Cells.Item(n));
            end
            series.Values = valRange;
        end
        Workbook.ActiveChart.HasTitle = 1;
        Workbook.ActiveChart.ChartTitle.Text = 'Lateral Resolution (Axial)';
        % Set/update chart position
        chartShape1.Top = Sheet.get('Cells',numRows+2,1).Top;
        chartShape1.Left = Sheet.get('Cells',numRows+2,1).Left+10;
        
        % Longitudinal chart
        longCol = numel(colHeaders)+3;
        chartShape2 = Sheet.ChartObjects.Item(2);
        chartShape2.Select;
        % Set/update chart data
        seriesCollection = Workbook.ActiveChart.SeriesCollection;
        if seriesCollection.Count == 0
            % Create series
            for s = 1:numel(rowHeadersLong)
                seriesCollection.NewSeries;
            end
        end
        for s = 1:numel(rowHeadersLong)
            series = seriesCollection.Item(s);
            series.Name = rowHeadersLong{s};
            % X Data
            dateColumn = Sheet.get('Range',Sheet.get('Cells',3,1),Sheet.get('Cells',dateCell.Row,1));
            dateRange = dateColumn.Cells.Item(1);
            for n = 1+interval:interval:dateColumn.Cells.Count;
                dateRange = Excel.Union(dateRange,dateColumn.Cells.Item(n));
            end
            series.XValues = dateRange;
            % Y Data
            valColumn = Sheet.get('Range',Sheet.get('Cells',3,longCol+2),...
                Sheet.get('Cells',dateCell.Row+numel(rowHeadersLong)-1,longCol+2));
            valRange = valColumn.Cells.Item(s);
            for n = s+interval:interval:valColumn.Cells.Count;
                valRange = Excel.Union(valRange,valColumn.Cells.Item(n));
            end
            series.Values = valRange;
        end
        Workbook.ActiveChart.HasTitle = 1;
        Workbook.ActiveChart.ChartTitle.Text = 'Lateral Resolution (Longitudinal)';
        % Set/update chart position
        chartShape2.Top = Sheet.get('Cells',numRows+2,chartShape1.BottomRightCell.Column+1).Top;
        chartShape2.Left = Sheet.get('Cells',numRows+2,chartShape1.BottomRightCell.Column+1).Left+10;
        
        % Save the workbook
        invoke(Workbook, 'Save');
        msgbox('Export successful.');
    else
        % Don't have write access, file may be open in another program
        errordlg('Cannot export to excel file. The file may be open in another application.',...
            'Error');
    end
    % Close Excel
    invoke(Excel, 'Quit');
catch exception
    disp(getReport(exception));
    % Make sure to close excel if error occurs
    invoke(Excel, 'Quit');
end


% --- Executes on button press in axialDistance_button_export.
function axialDistance_button_export_Callback(hObject, eventdata, handles)
% hObject    handle to axialResolution_button_export (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
rowHeaders = get(handles.axialDistance_table,'RowName');
colHeaders = get(handles.axialDistance_table,'ColumnName');
colHeaders = [colHeaders; 'Image'];
tableData = get(handles.axialDistance_table,'Data');
exportDate = datestr(clock,'yyyymmdd_HHMMSS');
exportFolder = fullfile(pwd,'Log\Images',date);
if ~exist(exportFolder,'dir')
    mkdir(exportFolder);
end

try
    % Get handle to Excel COM Server
    Excel = actxserver('Excel.Application');
    Excel.DisplayAlerts = 0;
    % Open Workbook
    Workbooks = Excel.Workbooks;
    Workbook = Open(Workbooks,fullfile(pwd,'Log/','ProstateBrachyQA Log.xlsx'));
    
    if Workbook.ReadOnly == 0
        % Have write access to excel file
        % Get a handle to Sheets
        Sheets = Excel.ActiveWorkBook.Sheets;
        Sheet = [];
        for sheetNum = 1:Sheets.Count
            % Get existing Axial Distance sheet
            if strcmp(Sheets.Item(sheetNum).Name,'Axial Distance')
                Sheet = get(Sheets, 'Item', sheetNum);
                break
            end
        end
        if isempty(Sheet)
            % Create new sheet after 5th sheet or last sheet
            if Sheets.Count >= 5
                Sheet = Sheets.Add([],Sheets.Item(5));
            else
                Sheet = Sheets.Add([],Sheets.Item(Sheets.Count));
            end
        end
        Sheet.Name = 'Axial Distance';
        Sheet.Activate;
        % Get number of last used row
        lastRow = Sheet.get('Cells').Find('*',Sheet.get('Cells',1,1),[],[],1,2);
        if ~isempty(lastRow)
            numRows = lastRow.Row;
        else
            % No data, initialize headers
            % Put test title in first cell
            Sheet.get('Cells',1,1).Value = 'Axial Distance';
            % Colour first cell yellow
            Sheet.get('Cells',1,1).Interior.ColorIndex = 6;
            % Column headers
            headers = Sheet.get('Range',Sheet.get('Cells',1,3),Sheet.get('Cells',1,numel(colHeaders)+2));
            headers.Value = colHeaders';
            numRows = 1;
            % Set first row to bold
            Sheet.Range('1:1').Font.Bold = 1;
            % Freeze first row
            Sheet.Application.ActiveWindow.SplitRow = 1;
            Sheet.Application.ActiveWindow.FreezePanes = true;
            % Set first column to bold
            Sheet.Range('A:A').Font.Bold = 1;
        end
        numCols = Sheet.get('Cells').Find('*',Sheet.get('Cells',1,1),[],[],2,2).Column;
        xlData = Sheet.UsedRange.Value(1:numRows,1:numCols);
        
        % Write new data
        if numRows > 1
            % Add extra space between dates
            numRows = numRows + 2;
        else
            % First entry, no need to add extra space
            numRows = numRows + 1;
        end
        % Write date
        dateCell = Sheet.get('Cells',numRows,1);
        dateCell.Value = date;
        
        % Write row headers
        rowHeaderRange = Sheet.get('Range',Sheet.get('Cells',dateCell.Row,2),Sheet.get('Cells',dateCell.Row+numel(rowHeaders)-1,2));
        rowHeaderRange.Value = rowHeaders;
        rowHeaderRange.Font.Bold = 1;
        % Write values
        for m = 1:numel(rowHeaders)
            rowNum = dateCell.Row + m - 1;
            for n = 1:numel(colHeaders)-1
                field = colHeaders{n};
                val = tableData{m,n};
                % Remove any html formatting
                val = regexprep(val, '<.*?>','');
                [~,fieldCol] = find(strcmp(xlData(1,:),field),1);
                if ~isempty(fieldCol)
                    % Column exists, write the value in new row
                    Sheet.get('Cells',rowNum,fieldCol).Value = val;
                end
            end
        end
        % Save image and add to excel sheet
        im = handles.exportImages{handles.testNum}{1};
        if ~exist(fullfile(pwd,'Log\Images'),'dir')
            mkdir(fullfile(pwd,'Log\Images'));
        end
        filename = fullfile(exportFolder,[exportDate '_AxialDistance.bmp']);
        imwrite(im,filename);
        [~,imageCol] = find(strcmp(xlData(1,:),'Image'),1);
        imageCell = Sheet.get('Cells',dateCell.Row,imageCol);
        % Add link to image
        Sheet.Hyperlinks.Add(imageCell,filename,[],[],'View Image');
        % Create comment for image preview on mouse hover
        imageCell.AddComment;
        imageCell.Comment.Shape.Fill.UserPicture(filename);
        imageCell.Comment.Shape.Width = size(im,2)/2;
        imageCell.Comment.Shape.Height = size(im,1)/2;
        
        % Update number of rows
        numRows = Sheet.get('Cells').Find('*',Sheet.get('Cells',1,1),[],[],1,2).Row;
        
        % Autofit columns
        Sheet.UsedRange.Columns.AutoFit;
        
        % Create/modify chart
        if Sheet.ChartObjects.Count == 0
            % If no chart exists, create one
            chartShape = Sheet.Shapes.AddChart;
            chartShape.Select;
            Workbook.ActiveChart.ChartType = 'xlXYScatterLines';
            Workbook.ActiveChart.Axes(1).TickLabels.Orientation = 35;
            % Clear default data
            Workbook.ActiveChart.ChartArea.ClearContents;
        end
        
        % Number of rows between measurements of different dates
        interval = 3;
        
        chartShape1 = Sheet.ChartObjects.Item(1);
        chartShape1.Select;
        % Set/update chart data
        seriesCollection = Workbook.ActiveChart.SeriesCollection;
        if seriesCollection.Count == 0
            % Create series
            for s = 1:numel(rowHeaders)
                seriesCollection.NewSeries;
            end
        end
        for s = 1:numel(rowHeaders)
            series = seriesCollection.Item(s);
            series.Name = rowHeaders{s};
            % X Data
            dateColumn = Sheet.get('Range',Sheet.get('Cells',2,1),Sheet.get('Cells',dateCell.Row,1));
            dateRange = dateColumn.Cells.Item(1);
            for n = 1+interval:interval:dateColumn.Cells.Count;
                dateRange = Excel.Union(dateRange,dateColumn.Cells.Item(n));
            end
            series.XValues = dateRange;
            % Y Data
            valColumn = Sheet.get('Range',Sheet.get('Cells',2,4),...
                Sheet.get('Cells',dateCell.Row+numel(rowHeaders)-1,4));
            valRange = valColumn.Cells.Item(s);
            for n = s+interval:interval:valColumn.Cells.Count;
                valRange = Excel.Union(valRange,valColumn.Cells.Item(n));
            end
            series.Values = valRange;
        end
        Workbook.ActiveChart.HasTitle = 1;
        Workbook.ActiveChart.ChartTitle.Text = 'Axial Distance';
        % Set/update chart position
        chartShape1.Top = Sheet.get('Cells',numRows+2,1).Top;
        chartShape1.Left = Sheet.get('Cells',numRows+2,1).Left+10;
        
        % Save the workbook
        invoke(Workbook, 'Save');
        msgbox('Export successful.');
    else
        % Don't have write access, file may be open in another program
        errordlg('Cannot export to excel file. The file may be open in another application.',...
            'Error');
    end
    % Close Excel
    invoke(Excel, 'Quit');
catch exception
    disp(getReport(exception));
    % Make sure to close excel if error occurs
    invoke(Excel, 'Quit');
end


% --- Executes on button press in lateralDistance_button_export.
function lateralDistance_button_export_Callback(hObject, eventdata, handles)
% hObject    handle to axialResolution_button_export (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
rowHeadersAxial = get(handles.lateralDistance_table_axial,'RowName');
rowHeadersLong = get(handles.lateralDistance_table_long,'RowName');
colHeaders = get(handles.lateralDistance_table_axial,'ColumnName');
colHeaders = [colHeaders; 'Image'];
tableDataAxial = get(handles.lateralDistance_table_axial,'Data');
tableDataLong = get(handles.lateralDistance_table_long,'Data');
exportDate = datestr(clock,'yyyymmdd_HHMMSS');
exportFolder = fullfile(pwd,'Log\Images',date);
if ~exist(exportFolder,'dir')
    mkdir(exportFolder);
end

try
    % Get handle to Excel COM Server
    Excel = actxserver('Excel.Application');
    Excel.DisplayAlerts = 0;
    % Open Workbook
    Workbooks = Excel.Workbooks;
    Workbook = Open(Workbooks,fullfile(pwd,'Log/','ProstateBrachyQA Log.xlsx'));
    
    if Workbook.ReadOnly == 0
        % Have write access to excel file
        % Get a handle to Sheets
        Sheets = Excel.ActiveWorkBook.Sheets;
        Sheet = [];
        for sheetNum = 1:Sheets.Count
            % Get existing Lateral Distance sheet
            if strcmp(Sheets.Item(sheetNum).Name,'Lateral Distance')
                Sheet = get(Sheets, 'Item', sheetNum);
                break
            end
        end
        if isempty(Sheet)
            % Create new sheet after 6th sheet or last sheet
            if Sheets.Count >= 6
                Sheet = Sheets.Add([],Sheets.Item(6));
            else
                Sheet = Sheets.Add([],Sheets.Item(Sheets.Count));
            end
        end
        Sheet.Name = 'Lateral Distance';
        Sheet.Activate;
        % Get number of last used row
        lastRow = Sheet.get('Cells').Find('*',Sheet.get('Cells',1,1),[],[],1,2);
        if ~isempty(lastRow)
            numRows = lastRow.Row;
        else
            % No data, initialize headers
            % Put test title in first cell
            Sheet.get('Cells',1,1).Value = 'Lateral Distance';
            % Colour first cell yellow
            Sheet.get('Cells',1,1).Interior.ColorIndex = 6;
            % Axial plane headers
            titleAxial = Sheet.get('Range',Sheet.get('Cells',1,2),Sheet.get('Cells',1,numel(colHeaders)+2));
            titleAxial.MergeCells = 1;
            titleAxial.Value = 'Axial Plane';
            headersAxial = Sheet.get('Range',Sheet.get('Cells',2,3),Sheet.get('Cells',2,numel(colHeaders)+2));
            headersAxial.Value = colHeaders';
            % Longitudinal plane headers
            longCol = headersAxial.End('xlToRight').Column + 1;
            titleLong = Sheet.get('Range',Sheet.get('Cells',1,longCol),Sheet.get('Cells',1,longCol+numel(colHeaders)-1));
            titleLong.MergeCells = 1;
            titleLong.Value = 'Longitudinal Plane';
            headersLong = Sheet.get('Range',Sheet.get('Cells',2,longCol+1),Sheet.get('Cells',2,longCol+numel(colHeaders)));
            headersLong.Value = colHeaders';
            numRows = 2;
            % Set first 2 rows to bold
            Sheet.Range('1:2').Font.Bold = 1;
            % Freeze first 2 rows
            Sheet.Application.ActiveWindow.SplitRow = 2;
            Sheet.Application.ActiveWindow.FreezePanes = true;
            % Set first column to bold
            Sheet.Range('A:A').Font.Bold = 1;
        end
        numCols = Sheet.get('Cells').Find('*',Sheet.get('Cells',1,1),[],[],2,2).Column;
        xlData = Sheet.UsedRange.Value(1:numRows,1:numCols);
        
        % Write new data
        if numRows > 2
            % Add extra space between dates
            numRows = numRows + 2;
        else
            % First entry, no need to add extra space
            numRows = numRows + 1;
        end
        % Write date
        dateCell = Sheet.get('Cells',numRows,1);
        dateCell.Value = date;
        
        % Axial plane
        % Write row headers
        rowHeaderRange = Sheet.get('Range',Sheet.get('Cells',dateCell.Row,2),Sheet.get('Cells',dateCell.Row+numel(rowHeadersAxial)-1,2));
        rowHeaderRange.Value = rowHeadersAxial;
        rowHeaderRange.Font.Bold = 1;
        % Write values
        for m = 1:numel(rowHeadersAxial)
            rowNum = dateCell.Row + m - 1;
            for n = 1:numel(colHeaders)-1
                field = colHeaders{n};
                val = tableDataAxial{m,n};
                % Remove any html formatting
                val = regexprep(val, '<.*?>','');
                [~,fieldCol] = find(strcmp(xlData(2,:),field),1);
                if ~isempty(fieldCol)
                    % Column exists, write the value in new row
                    Sheet.get('Cells',rowNum,fieldCol).Value = val;
                end
            end
        end
        % Save axial image and add to excel sheet
        imAxial = handles.exportImages{handles.testNum}{1};
        if ~exist(fullfile(pwd,'Log\Images'),'dir')
            mkdir(fullfile(pwd,'Log\Images'));
        end
        filename = fullfile(exportFolder,[exportDate '_LateralDistance_Axial.bmp']);
        imwrite(imAxial,filename);
        [~,imageCol] = find(strcmp(xlData(2,:),'Image'),1);
        imageCell = Sheet.get('Cells',dateCell.Row,imageCol);
        % Add link to image
        Sheet.Hyperlinks.Add(imageCell,filename,[],[],'View Image');
        % Create comment for image preview on mouse hover
        imageCell.AddComment;
        imageCell.Comment.Shape.Fill.UserPicture(filename);
        imageCell.Comment.Shape.Width = size(imAxial,2)/2;
        imageCell.Comment.Shape.Height = size(imAxial,1)/2;
        
        % Longitudinal plane
        longCol = numel(colHeaders)+3;
        % Write row headers
        rowHeaderRange = Sheet.get('Range',Sheet.get('Cells',dateCell.Row,longCol),Sheet.get('Cells',dateCell.Row+numel(rowHeadersLong)-1,longCol));
        rowHeaderRange.Value = rowHeadersLong;
        rowHeaderRange.Font.Bold = 1;
        % Write values
        for m = 1:numel(rowHeadersLong)
            rowNum = dateCell.Row + m - 1;
            for n = 1:numel(colHeaders)-1
                field = colHeaders{n};
                val = tableDataLong{m,n};
                % Remove any html formatting
                val = regexprep(val, '<.*?>','');
                [~,fieldCol] = find(strcmp(xlData(2,:),field),1,'last');
                if ~isempty(fieldCol)
                    % Column exists, write the value in new row
                    Sheet.get('Cells',rowNum,fieldCol).Value = val;
                end
            end
        end
        % Save longitudinal image and add to excel sheet
        imLong = handles.exportImages{handles.testNum}{2};
        if ~exist(fullfile(pwd,'Log\Images'),'dir')
            mkdir(fullfile(pwd,'Log\Images'));
        end
        filename = fullfile(exportFolder,[exportDate '_LateralDistance_Longitudinal.bmp']);
        imwrite(imLong,filename);
        [~,imageCol] = find(strcmp(xlData(2,:),'Image'),1,'last');
        imageCell = Sheet.get('Cells',dateCell.Row,imageCol);
        % Add link to image
        Sheet.Hyperlinks.Add(imageCell,filename,[],[],'View Image');
        % Create comment for image preview on mouse hover
        imageCell.AddComment;
        imageCell.Comment.Shape.Fill.UserPicture(filename);
        imageCell.Comment.Shape.Width = size(imLong,2)/2;
        imageCell.Comment.Shape.Height = size(imLong,1)/2;
        
        % Update number of rows
        numRows = Sheet.get('Cells').Find('*',Sheet.get('Cells',1,1),[],[],1,2).Row;
        
        % Autofit columns
        Sheet.UsedRange.Columns.AutoFit;
        
        % Create/modify chart
        if Sheet.ChartObjects.Count == 0
            % If no chart exists, create them
            for n = 1:2
                chartShape = Sheet.Shapes.AddChart;
                chartShape.Select;
                Workbook.ActiveChart.ChartType = 'xlXYScatterLines';
                Workbook.ActiveChart.Axes(1).TickLabels.Orientation = 35;
                % Clear default data
                Workbook.ActiveChart.ChartArea.ClearContents;
            end
        end
        
        % Number of rows between measurements of different dates
        interval = 3;
        
        % Axial chart
        axialCol = 2;
        chartShape1 = Sheet.ChartObjects.Item(1);
        chartShape1.Select;
        % Set/update chart data
        seriesCollection = Workbook.ActiveChart.SeriesCollection;
        if seriesCollection.Count == 0
            % Create series
            for s = 1:numel(rowHeadersAxial)
                seriesCollection.NewSeries;
            end
        end
        for s = 1:numel(rowHeadersAxial)
            series = seriesCollection.Item(s);
            series.Name = rowHeadersAxial{s};
            % X Data
            dateColumn = Sheet.get('Range',Sheet.get('Cells',3,1),Sheet.get('Cells',dateCell.Row,1));
            dateRange = dateColumn.Cells.Item(1);
            for n = 1+interval:interval:dateColumn.Cells.Count;
                dateRange = Excel.Union(dateRange,dateColumn.Cells.Item(n));
            end
            series.XValues = dateRange;
            % Y Data
            valColumn = Sheet.get('Range',Sheet.get('Cells',3,axialCol+2),...
                Sheet.get('Cells',dateCell.Row+numel(rowHeadersAxial)-1,axialCol+2));
            valRange = valColumn.Cells.Item(s);
            for n = s+interval:interval:valColumn.Cells.Count;
                valRange = Excel.Union(valRange,valColumn.Cells.Item(n));
            end
            series.Values = valRange;
        end
        Workbook.ActiveChart.HasTitle = 1;
        Workbook.ActiveChart.ChartTitle.Text = 'Lateral Distance (Axial)';
        % Set/update chart position
        chartShape1.Top = Sheet.get('Cells',numRows+2,1).Top;
        chartShape1.Left = Sheet.get('Cells',numRows+2,1).Left+10;
        
        % Longitudinal chart
        longCol = numel(colHeaders)+3;
        chartShape2 = Sheet.ChartObjects.Item(2);
        chartShape2.Select;
        % Set/update chart data
        seriesCollection = Workbook.ActiveChart.SeriesCollection;
        if seriesCollection.Count == 0
            % Create series
            for s = 1:numel(rowHeadersLong)
                seriesCollection.NewSeries;
            end
        end
        for s = 1:numel(rowHeadersLong)
            series = seriesCollection.Item(s);
            series.Name = rowHeadersLong{s};
            % X Data
            dateColumn = Sheet.get('Range',Sheet.get('Cells',3,1),Sheet.get('Cells',dateCell.Row,1));
            dateRange = dateColumn.Cells.Item(1);
            for n = 1+interval:interval:dateColumn.Cells.Count;
                dateRange = Excel.Union(dateRange,dateColumn.Cells.Item(n));
            end
            series.XValues = dateRange;
            % Y Data
            valColumn = Sheet.get('Range',Sheet.get('Cells',3,longCol+2),...
                Sheet.get('Cells',dateCell.Row+numel(rowHeadersLong)-1,longCol+2));
            valRange = valColumn.Cells.Item(s);
            for n = s+interval:interval:valColumn.Cells.Count;
                valRange = Excel.Union(valRange,valColumn.Cells.Item(n));
            end
            series.Values = valRange;
        end
        Workbook.ActiveChart.HasTitle = 1;
        Workbook.ActiveChart.ChartTitle.Text = 'Lateral Distance (Longitudinal)';
        % Set/update chart position
        chartShape2.Top = Sheet.get('Cells',numRows+2,chartShape1.BottomRightCell.Column+1).Top;
        chartShape2.Left = Sheet.get('Cells',numRows+2,chartShape1.BottomRightCell.Column+1).Left+10;
        
        % Save the workbook
        invoke(Workbook, 'Save');
        msgbox('Export successful.');
    else
        % Don't have write access, file may be open in another program
        errordlg('Cannot export to excel file. The file may be open in another application.',...
            'Error');
    end
    % Close Excel
    invoke(Excel, 'Quit');
catch exception
    disp(getReport(exception));
    % Make sure to close excel if error occurs
    invoke(Excel, 'Quit');
end


% --- Executes on button press in area_button_export.
function area_button_export_Callback(hObject, eventdata, handles)
% hObject    handle to axialResolution_button_export (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
rowHeaders = get(handles.area_table,'RowName');
colHeaders = get(handles.area_table,'ColumnName');
colHeaders = [colHeaders; 'Image'];
tableData = get(handles.area_table,'Data');
exportDate = datestr(clock,'yyyymmdd_HHMMSS');
exportFolder = fullfile(pwd,'Log\Images',date);
if ~exist(exportFolder,'dir')
    mkdir(exportFolder);
end

% Remove formatting from column headers
for h = 1:numel(colHeaders)
    % Replace <sup> with ^
    colHeaders{h} = regexprep(colHeaders{h}, '<sup>','^');
    % Remove any html formatting
    colHeaders{h} = regexprep(colHeaders{h}, '<.*?>','');
end

try
    % Get handle to Excel COM Server
    Excel = actxserver('Excel.Application');
    Excel.DisplayAlerts = 0;
    % Open Workbook
    Workbooks = Excel.Workbooks;
    Workbook = Open(Workbooks,fullfile(pwd,'Log/','ProstateBrachyQA Log.xlsx'));
    
    if Workbook.ReadOnly == 0
        % Have write access to excel file
        % Get a handle to Sheets
        Sheets = Excel.ActiveWorkBook.Sheets;
        Sheet = [];
        for sheetNum = 1:Sheets.Count
            % Get existing Area sheet
            if strcmp(Sheets.Item(sheetNum).Name,'Area')
                Sheet = get(Sheets, 'Item', sheetNum);
                break
            end
        end
        if isempty(Sheet)
            % Create new sheet after 7th sheet or last sheet
            if Sheets.Count >= 7
                Sheet = Sheets.Add([],Sheets.Item(7));
            else
                Sheet = Sheets.Add([],Sheets.Item(Sheets.Count));
            end
        end
        Sheet.Name = 'Area';
        Sheet.Activate;
        % Get number of last used row
        lastRow = Sheet.get('Cells').Find('*',Sheet.get('Cells',1,1),[],[],1,2);
        if ~isempty(lastRow)
            numRows = lastRow.Row;
        else
            % No data, initialize headers
            % Put test title in first cell
            Sheet.get('Cells',1,1).Value = 'Area';
            % Colour first cell yellow
            Sheet.get('Cells',1,1).Interior.ColorIndex = 6;
            % Column headers
            headers = Sheet.get('Range',Sheet.get('Cells',1,2),Sheet.get('Cells',1,numel(colHeaders)+1));
            headers.Value = colHeaders';
            numRows = 1;
            % Set first row to bold
            Sheet.Range('1:1').Font.Bold = 1;
            % Freeze first row
            Sheet.Application.ActiveWindow.SplitRow = 1;
            Sheet.Application.ActiveWindow.FreezePanes = true;
            % Set first column to bold
            Sheet.Range('A:A').Font.Bold = 1;
        end
        numCols = Sheet.get('Cells').Find('*',Sheet.get('Cells',1,1),[],[],2,2).Column;
        xlData = Sheet.UsedRange.Value(1:numRows,1:numCols);
        
        numRows = numRows + 1;
        % Write date
        dateCell = Sheet.get('Cells',numRows,1);
        dateCell.Value = date;
        
        % Write values
        for m = 1:numel(rowHeaders)
            rowNum = dateCell.Row + m - 1;
            for n = 1:numel(colHeaders)-1
                field = colHeaders{n};
                val = tableData{m,n};
                % Remove any html formatting
                val = regexprep(val, '<.*?>','');
                [~,fieldCol] = find(strcmp(xlData(1,:),field),1);
                if ~isempty(fieldCol)
                    % Column exists, write the value in new row
                    Sheet.get('Cells',rowNum,fieldCol).Value = val;
                end
            end
        end
        % Save image and add to excel sheet
        im = handles.exportImages{handles.testNum}{1};
        if ~exist(fullfile(pwd,'Log\Images'),'dir')
            mkdir(fullfile(pwd,'Log\Images'));
        end
        filename = fullfile(exportFolder,[exportDate '_Area.bmp']);
        imwrite(im,filename);
        [~,imageCol] = find(strcmp(xlData(1,:),'Image'),1);
        imageCell = Sheet.get('Cells',dateCell.Row,imageCol);
        % Add link to image
        Sheet.Hyperlinks.Add(imageCell,filename,[],[],'View Image');
        % Create comment for image preview on mouse hover
        imageCell.AddComment;
        imageCell.Comment.Shape.Fill.UserPicture(filename);
        imageCell.Comment.Shape.Width = size(im,2)/2;
        imageCell.Comment.Shape.Height = size(im,1)/2;
        
        % Update number of rows
        numRows = Sheet.get('Cells').Find('*',Sheet.get('Cells',1,1),[],[],1,2).Row;
        
        % Autofit columns
        Sheet.UsedRange.Columns.AutoFit;
        
        % Create/modify chart
        if Sheet.ChartObjects.Count == 0
            % If no chart exists, create one
            chartShape = Sheet.Shapes.AddChart;
            chartShape.Select;
            Workbook.ActiveChart.ChartType = 'xlXYScatterLines';
            Workbook.ActiveChart.Axes(1).TickLabels.Orientation = 35;
        else
            % Select existing chart
            chartShape = Sheet.ChartObjects.Item(1);
            chartShape.Select;
        end
        % Set/update chart data
        knownValsCol = Sheet.get('Range',Sheet.get('Cells',2,2),Sheet.get('Cells',numRows,2)).Value;
        if iscell(knownValsCol)
            knownValsCol = cell2mat(knownValsCol);
        end
        knownVals = unique(knownValsCol);
        seriesCollection = Workbook.ActiveChart.SeriesCollection;
        for s = 1:numel(knownVals)
            if seriesCollection.Count < s
                % Create series
                seriesCollection.NewSeries;
            end
            series = seriesCollection.Item(s);
            series.Name = ['Known Value: ' num2str(knownVals(s))];
            % Get row numbers for this known value
            rowNums = find(knownValsCol==knownVals(s));
            % X Data
            % Get test dates for this known value
            dateColumn = Sheet.get('Range',Sheet.get('Cells',2,1),Sheet.get('Cells',numRows,1));
            dateRange = dateColumn.Cells.Item(rowNums(1));
            for n = 2:numel(rowNums)
                dateRange = Excel.Union(dateRange,dateColumn.Cells.Item(rowNums(n)));
            end
            series.XValues = dateRange;
            % Y Data
            valColumn = Sheet.get('Range',Sheet.get('Cells',2,3),Sheet.get('Cells',numRows,3));
            valRange = valColumn.Cells.Item(rowNums(1));
            for n = 2:numel(rowNums)
                valRange = Excel.Union(valRange,valColumn.Cells.Item(rowNums(n)));
            end
            series.Values = valRange;
        end
        Workbook.ActiveChart.HasTitle = 1;
        Workbook.ActiveChart.ChartTitle.Text = 'Area';
        % Set/update chart position
        chartShape.Top = Sheet.get('Cells',numRows+2,1).Top;
        chartShape.Left = Sheet.get('Cells',numRows+2,1).Left+10;
        
        % Save the workbook
        invoke(Workbook, 'Save');
        msgbox('Export successful.');
    else
        % Don't have write access, file may be open in another program
        errordlg('Cannot export to excel file. The file may be open in another application.',...
            'Error');
    end
    % Close Excel
    invoke(Excel, 'Quit');
catch exception
    disp(getReport(exception));
    % Make sure to close excel if error occurs
    invoke(Excel, 'Quit');
end


% --- Executes on button press in volume_button_export.
function volume_button_export_Callback(hObject, eventdata, handles)
% hObject    handle to volume_button_export (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
rowHeaders = get(handles.volume_table,'RowName');
colHeaders = get(handles.volume_table,'ColumnName');
colHeaders = [colHeaders; 'Images'];
tableData = get(handles.volume_table,'Data');
exportDate = datestr(clock,'yyyymmdd_HHMMSS');
exportFolder = fullfile(pwd,'Log\Images',date);
if ~exist(exportFolder,'dir')
    mkdir(exportFolder);
end

% Remove formatting from column headers
for h = 1:numel(colHeaders)
    % Replace <sup> with ^
    colHeaders{h} = regexprep(colHeaders{h}, '<sup>','^');
    % Remove any html formatting
    colHeaders{h} = regexprep(colHeaders{h}, '<.*?>','');
end

try
    % Get handle to Excel COM Server
    Excel = actxserver('Excel.Application');
    Excel.DisplayAlerts = 0;
    % Open Workbook
    Workbooks = Excel.Workbooks;
    Workbook = Open(Workbooks,fullfile(pwd,'Log/','ProstateBrachyQA Log.xlsx'));
    
    if Workbook.ReadOnly == 0
        % Have write access to excel file
        % Get a handle to Sheets
        Sheets = Excel.ActiveWorkBook.Sheets;
        Sheet = [];
        for sheetNum = 1:Sheets.Count
            % Get existing Volume sheet
            if strcmp(Sheets.Item(sheetNum).Name,'Volume')
                Sheet = get(Sheets, 'Item', sheetNum);
                break
            end
        end
        if isempty(Sheet)
            % Create new sheet after 8th sheet or last sheet
            if Sheets.Count >= 8
                Sheet = Sheets.Add([],Sheets.Item(8));
            else
                Sheet = Sheets.Add([],Sheets.Item(Sheets.Count));
            end
        end
        Sheet.Name = 'Volume';
        Sheet.Activate;
        % Get number of last used row
        lastRow = Sheet.get('Cells').Find('*',Sheet.get('Cells',1,1),[],[],1,2);
        if ~isempty(lastRow)
            numRows = lastRow.Row;
        else
            % No data, initialize headers
            % Put test title in first cell
            Sheet.get('Cells',1,1).Value = 'Volume';
            % Colour first cell yellow
            Sheet.get('Cells',1,1).Interior.ColorIndex = 6;
            % Column headers
            headers = Sheet.get('Range',Sheet.get('Cells',1,2),Sheet.get('Cells',1,numel(colHeaders)+1));
            headers.Value = colHeaders';
            numRows = 1;
            % Set first row to bold
            Sheet.Range('1:1').Font.Bold = 1;
            % Freeze first row
            Sheet.Application.ActiveWindow.SplitRow = 1;
            Sheet.Application.ActiveWindow.FreezePanes = true;
            % Set first column to bold
            Sheet.Range('A:A').Font.Bold = 1;
        end
        numCols = Sheet.get('Cells').Find('*',Sheet.get('Cells',1,1),[],[],2,2).Column;
        xlData = Sheet.UsedRange.Value(1:numRows,1:numCols);
        
        numRows = numRows + 1;
        % Write date
        dateCell = Sheet.get('Cells',numRows,1);
        dateCell.Value = date;
        
        % Write values
        for m = 1:numel(rowHeaders)
            rowNum = dateCell.Row + m - 1;
            for n = 1:numel(colHeaders)-1
                field = colHeaders{n};
                val = tableData{m,n};
                % Remove any html formatting
                val = regexprep(val, '<.*?>','');
                [~,fieldCol] = find(strcmp(xlData(1,:),field),1);
                if ~isempty(fieldCol)
                    % Column exists, write the value in new row
                    Sheet.get('Cells',rowNum,fieldCol).Value = val;
                end
            end
        end
        % Save images and add to excel sheet
        % Create figure for copying axes onto and saving image
        fig = figure('Visible','off');
        for imNum = 1:numel(handles.volume_axes_list)
            im = getAxesImage(handles.volume_axes_list(imNum),fig);
            handles.exportImages{handles.testNum}{imNum} = im;
            filename = fullfile(exportFolder,[exportDate '_Volume_' num2str(imNum) '.bmp']);
            imwrite(im,filename);
            [~,imageCol] = find(strcmp(xlData(1,:),'Images'),1);
            imageCell = Sheet.get('Cells',dateCell.Row,imageCol+imNum-1);
            % Add link to image
            Sheet.Hyperlinks.Add(imageCell,filename,[],[],['View Image ' num2str(imNum)]);
            % Create comment for image preview on mouse hover
            imageCell.AddComment;
            imageCell.Comment.Shape.Fill.UserPicture(filename);
            imageCell.Comment.Shape.Width = size(im,2)/2;
            imageCell.Comment.Shape.Height = size(im,1)/2;
        end
        % Delete the copied plots figure
        delete(fig);
        
        % Update number of rows
        numRows = Sheet.get('Cells').Find('*',Sheet.get('Cells',1,1),[],[],1,2).Row;
        
        % Autofit columns
        Sheet.UsedRange.Columns.AutoFit;
        
        % Create/modify chart
        if Sheet.ChartObjects.Count == 0
            % If no chart exists, create one
            chartShape = Sheet.Shapes.AddChart;
            chartShape.Select;
            Workbook.ActiveChart.ChartType = 'xlXYScatterLines';
            Workbook.ActiveChart.Axes(1).TickLabels.Orientation = 35;
        else
            % Select existing chart
            chartShape = Sheet.ChartObjects.Item(1);
            chartShape.Select;
        end
        % Set/update chart data
        knownValsCol = Sheet.get('Range',Sheet.get('Cells',2,2),Sheet.get('Cells',numRows,2)).Value;
        if iscell(knownValsCol)
            knownValsCol = cell2mat(knownValsCol);
        end
        knownVals = unique(knownValsCol);
        seriesCollection = Workbook.ActiveChart.SeriesCollection;
        for s = 1:numel(knownVals)
            if seriesCollection.Count < s
                % Create series
                seriesCollection.NewSeries;
            end
            series = seriesCollection.Item(s);
            series.Name = ['Known Value: ' num2str(knownVals(s))];
            % Get row numbers for this known value
            rowNums = find(knownValsCol==knownVals(s));
            % X Data
            % Get test dates for this known value
            dateColumn = Sheet.get('Range',Sheet.get('Cells',2,1),Sheet.get('Cells',numRows,1));
            dateRange = dateColumn.Cells.Item(rowNums(1));
            for n = 2:numel(rowNums)
                dateRange = Excel.Union(dateRange,dateColumn.Cells.Item(rowNums(n)));
            end
            series.XValues = dateRange;
            % Y Data
            valColumn = Sheet.get('Range',Sheet.get('Cells',2,3),Sheet.get('Cells',numRows,3));
            valRange = valColumn.Cells.Item(rowNums(1));
            for n = 2:numel(rowNums)
                valRange = Excel.Union(valRange,valColumn.Cells.Item(rowNums(n)));
            end
            series.Values = valRange;
        end
        Workbook.ActiveChart.HasTitle = 1;
        Workbook.ActiveChart.ChartTitle.Text = 'Volume';
        % Set/update chart position
        chartShape.Top = Sheet.get('Cells',numRows+2,1).Top;
        chartShape.Left = Sheet.get('Cells',numRows+2,1).Left+10;
        
        % Save the workbook
        invoke(Workbook, 'Save');
        msgbox('Export successful.');
    else
        % Don't have write access, file may be open in another program
        errordlg('Cannot export to excel file. The file may be open in another application.',...
            'Error');
    end
    % Close Excel
    invoke(Excel, 'Quit');
catch exception
    disp(getReport(exception));
    % Make sure to close excel if error occurs
    invoke(Excel, 'Quit');
end


% --- Executes on button press in volumeFormula_button_export.
function volumeFormula_button_export_Callback(hObject, eventdata, handles)
% hObject    handle to volumeFormula_button_export (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
rowHeaders = get(handles.volumeFormula_table,'RowName');
colHeaders = get(handles.volumeFormula_table,'ColumnName');
colHeaders = [colHeaders; 'Images'];
tableData = get(handles.volumeFormula_table,'Data');
exportDate = datestr(clock,'yyyymmdd_HHMMSS');
exportFolder = fullfile(pwd,'Log\Images',date);
if ~exist(exportFolder,'dir')
    mkdir(exportFolder);
end

% Remove formatting from column headers
for h = 1:numel(colHeaders)
    % Replace <sup> with ^
    colHeaders{h} = regexprep(colHeaders{h}, '<sup>','^');
    % Remove any html formatting
    colHeaders{h} = regexprep(colHeaders{h}, '<.*?>','');
end

try
    % Get handle to Excel COM Server
    Excel = actxserver('Excel.Application');
    Excel.DisplayAlerts = 0;
    % Open Workbook
    Workbooks = Excel.Workbooks;
    Workbook = Open(Workbooks,fullfile(pwd,'Log/','ProstateBrachyQA Log.xlsx'));
    
    if Workbook.ReadOnly == 0
        % Have write access to excel file
        % Get a handle to Sheets
        Sheets = Excel.ActiveWorkBook.Sheets;
        Sheet = [];
        for sheetNum = 1:Sheets.Count
            % Get existing Volume sheet
            if strcmp(Sheets.Item(sheetNum).Name,'Volume')
                Sheet = get(Sheets, 'Item', sheetNum);
                break
            end
        end
        if isempty(Sheet)
            % Create new sheet after 8th sheet or last sheet
            if Sheets.Count >= 8
                Sheet = Sheets.Add([],Sheets.Item(8));
            else
                Sheet = Sheets.Add([],Sheets.Item(Sheets.Count));
            end
        end
        Sheet.Name = 'Volume';
        Sheet.Activate;
        % Get number of last used row
        lastRow = Sheet.get('Cells').Find('*',Sheet.get('Cells',1,1),[],[],1,2);
        if ~isempty(lastRow)
            numRows = lastRow.Row;
        else
            % No data, initialize headers
            % Put test title in first cell
            Sheet.get('Cells',1,1).Value = 'Volume';
            % Colour first cell yellow
            Sheet.get('Cells',1,1).Interior.ColorIndex = 6;
            % Column headers
            headers = Sheet.get('Range',Sheet.get('Cells',1,2),Sheet.get('Cells',1,numel(colHeaders)+1));
            headers.Value = colHeaders';
            numRows = 1;
            % Set first row to bold
            Sheet.Range('1:1').Font.Bold = 1;
            % Freeze first row
            Sheet.Application.ActiveWindow.SplitRow = 1;
            Sheet.Application.ActiveWindow.FreezePanes = true;
            % Set first column to bold
            Sheet.Range('A:A').Font.Bold = 1;
        end
        numCols = Sheet.get('Cells').Find('*',Sheet.get('Cells',1,1),[],[],2,2).Column;
        xlData = Sheet.UsedRange.Value(1:numRows,1:numCols);
        
        numRows = numRows + 1;
        % Write date
        dateCell = Sheet.get('Cells',numRows,1);
        dateCell.Value = date;
        
        % Write values
        for m = 1:numel(rowHeaders)
            rowNum = dateCell.Row + m - 1;
            for n = 1:numel(colHeaders)-1
                field = colHeaders{n};
                val = tableData{m,n};
                % Remove any html formatting
                val = regexprep(val, '<.*?>','');
                [~,fieldCol] = find(strcmp(xlData(1,:),field),1);
                if ~isempty(fieldCol)
                    % Column exists, write the value in new row
                    Sheet.get('Cells',rowNum,fieldCol).Value = val;
                end
            end
        end
        % Save images and add to excel sheet
        % Create figure for copying axes onto and saving image
        fig = figure('Visible','off');
        for imNum = 1:numel(handles.volumeFormula_axes_list)
            im = getAxesImage(handles.volumeFormula_axes_list(imNum),fig);
            handles.exportImages{handles.testNum}{imNum} = im;
            filename = fullfile(exportFolder,[exportDate '_Volume_' num2str(imNum) '.bmp']);
            imwrite(im,filename);
            [~,imageCol] = find(strcmp(xlData(1,:),'Images'),1);
            imageCell = Sheet.get('Cells',dateCell.Row,imageCol+imNum-1);
            % Add link to image
            Sheet.Hyperlinks.Add(imageCell,filename,[],[],['View Image ' num2str(imNum)]);
            % Create comment for image preview on mouse hover
            imageCell.AddComment;
            imageCell.Comment.Shape.Fill.UserPicture(filename);
            imageCell.Comment.Shape.Width = size(im,2)/2;
            imageCell.Comment.Shape.Height = size(im,1)/2;
        end
        % Delete the copied plots figure
        delete(fig);
        
        % Update number of rows
        numRows = Sheet.get('Cells').Find('*',Sheet.get('Cells',1,1),[],[],1,2).Row;
        
        % Autofit columns
        Sheet.UsedRange.Columns.AutoFit;
        
        % Create/modify chart
        if Sheet.ChartObjects.Count == 0
            % If no chart exists, create one
            chartShape = Sheet.Shapes.AddChart;
            chartShape.Select;
            Workbook.ActiveChart.ChartType = 'xlXYScatterLines';
            Workbook.ActiveChart.Axes(1).TickLabels.Orientation = 35;
        else
            % Select existing chart
            chartShape = Sheet.ChartObjects.Item(1);
            chartShape.Select;
        end
        % Set/update chart data
        knownValsCol = Sheet.get('Range',Sheet.get('Cells',2,2),Sheet.get('Cells',numRows,2)).Value;
        if iscell(knownValsCol)
            knownValsCol = cell2mat(knownValsCol);
        end
        knownVals = unique(knownValsCol);
        seriesCollection = Workbook.ActiveChart.SeriesCollection;
        for s = 1:numel(knownVals)
            if seriesCollection.Count < s
                % Create series
                seriesCollection.NewSeries;
            end
            series = seriesCollection.Item(s);
            series.Name = ['Known Value: ' num2str(knownVals(s))];
            % Get row numbers for this known value
            rowNums = find(knownValsCol==knownVals(s));
            % X Data
            % Get test dates for this known value
            dateColumn = Sheet.get('Range',Sheet.get('Cells',2,1),Sheet.get('Cells',numRows,1));
            dateRange = dateColumn.Cells.Item(rowNums(1));
            for n = 2:numel(rowNums)
                dateRange = Excel.Union(dateRange,dateColumn.Cells.Item(rowNums(n)));
            end
            series.XValues = dateRange;
            % Y Data
            valColumn = Sheet.get('Range',Sheet.get('Cells',2,3),Sheet.get('Cells',numRows,3));
            valRange = valColumn.Cells.Item(rowNums(1));
            for n = 2:numel(rowNums)
                valRange = Excel.Union(valRange,valColumn.Cells.Item(rowNums(n)));
            end
            series.Values = valRange;
        end
        Workbook.ActiveChart.HasTitle = 1;
        Workbook.ActiveChart.ChartTitle.Text = 'Volume';
        % Set/update chart position
        chartShape.Top = Sheet.get('Cells',numRows+2,1).Top;
        chartShape.Left = Sheet.get('Cells',numRows+2,1).Left+10;
        
        % Save the workbook
        invoke(Workbook, 'Save');
        msgbox('Export successful.');
    else
        % Don't have write access, file may be open in another program
        errordlg('Cannot export to excel file. The file may be open in another application.',...
            'Error');
    end
    % Close Excel
    invoke(Excel, 'Quit');
catch exception
    disp(getReport(exception));
    % Make sure to close excel if error occurs
    invoke(Excel, 'Quit');
end


% --- Executes on button press in gridAlignment_button_export.
function gridAlignment_button_export_Callback(hObject, eventdata, handles)
% hObject    handle to gridAlignment_button_export (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
rowHeaders = get(handles.gridAlignment_table,'RowName');
colHeaders = get(handles.gridAlignment_table,'ColumnName');
colHeaders = [colHeaders; 'Image'];
tableData = get(handles.gridAlignment_table,'Data');
rowHeaders2 = get(handles.gridAlignment_table_probeToG2,'RowName');
colHeaders2 = get(handles.gridAlignment_table_probeToG2,'ColumnName');
tableData2 = get(handles.gridAlignment_table_probeToG2,'Data');
exportDate = datestr(clock,'yyyymmdd_HHMMSS');
exportFolder = fullfile(pwd,'Log\Images',date);
if ~exist(exportFolder,'dir')
    mkdir(exportFolder);
end

try
    % Get handle to Excel COM Server
    Excel = actxserver('Excel.Application');
    Excel.DisplayAlerts = 0;
    % Open Workbook
    Workbooks = Excel.Workbooks;
    Workbook = Open(Workbooks,fullfile(pwd,'Log/','ProstateBrachyQA Log.xlsx'));
    
    if Workbook.ReadOnly == 0
        % Have write access to excel file
        % Get a handle to Sheets
        Sheets = Excel.ActiveWorkBook.Sheets;
        Sheet = [];
        for sheetNum = 1:Sheets.Count
            % Get existing Grid Alignment sheet
            if strcmp(Sheets.Item(sheetNum).Name,'Grid Alignment')
                Sheet = get(Sheets, 'Item', sheetNum);
                break
            end
        end
        if isempty(Sheet)
            % Create new sheet after 8th sheet or last sheet
            if Sheets.Count >= 9
                Sheet = Sheets.Add([],Sheets.Item(9));
            else
                Sheet = Sheets.Add([],Sheets.Item(Sheets.Count));
            end
        end
        Sheet.Name = 'Grid Alignment';
        Sheet.Activate;
        % Get number of last used row
        lastRow = Sheet.get('Cells').Find('*',Sheet.get('Cells',1,1),[],[],1,2);
        if ~isempty(lastRow)
            numRows = lastRow.Row;
        else
            % No data, initialize headers
            % Put test title in first cell
            Sheet.get('Cells',1,1).Value = 'Grid Alignment';
            % Colour first cell yellow
            Sheet.get('Cells',1,1).Interior.ColorIndex = 6;
            % Column headers
            % Grid coordinates headers
            headers = Sheet.get('Range',Sheet.get('Cells',1,3),Sheet.get('Cells',1,numel(colHeaders)+2));
            headers.Value = colHeaders';
            % Probe to G2 headers
            header2Col = headers.End('xlToRight').Column+2;
            headers2 = Sheet.get('Cells',1,header2Col);
            headers2.Value = colHeaders2;
            numRows = 1;
            % Set first row to bold
            Sheet.Range('1:1').Font.Bold = 1;
            % Freeze first row
            Sheet.Application.ActiveWindow.SplitRow = 1;
            Sheet.Application.ActiveWindow.FreezePanes = true;
            % Set first column to bold
            Sheet.Range('A:A').Font.Bold = 1;
        end
        numCols = Sheet.get('Cells').Find('*',Sheet.get('Cells',1,1),[],[],2,2).Column;
        xlData = Sheet.UsedRange.Value(1:numRows,1:numCols);
        
        % Write new data
        if numRows > 2
            % Add extra space between dates
            numRows = numRows + 2;
        else
            % First entry, no need to add extra space
            numRows = numRows + 1;
        end
        
        % Write date
        dateCell = Sheet.get('Cells',numRows,1);
        dateCell.Value = date;
        
        % Write row headers
        rowHeaderRange = Sheet.get('Range',Sheet.get('Cells',dateCell.Row,2),Sheet.get('Cells',dateCell.Row+numel(rowHeaders)-1,2));
        rowHeaderRange.Value = rowHeaders;
        rowHeaderRange.Font.Bold = 1;
        
        % Write values
        for m = 1:numel(rowHeaders)
            rowNum = dateCell.Row + m - 1;
            for n = 1:numel(colHeaders)-1
                field = colHeaders{n};
                val = tableData{m,n};
                % Remove any html formatting
                val = regexprep(val, '<.*?>','');
                [~,fieldCol] = find(strcmp(xlData(1,:),field),1);
                if ~isempty(fieldCol)
                    % Column exists, write the value in new row
                    Sheet.get('Cells',rowNum,fieldCol).Value = val;
                end
            end
        end
        % Save images and add to excel sheet
        % Create figure for copying axes onto and saving image
        fig = figure('Visible','off');
        for imNum = 1:numel(handles.gridAlignment_axes_list)
            im = getAxesImage(handles.gridAlignment_axes_list(imNum),fig);
            handles.exportImages{handles.testNum}{imNum} = im;
            filename = fullfile(exportFolder,[exportDate '_GridAlignment_' num2str(imNum) '.bmp']);
            imwrite(im,filename);
            [~,imageCol] = find(strcmp(xlData(1,:),'Image'),1);
            imageCell = Sheet.get('Cells',dateCell.Row+imNum-1,imageCol);
            % Add link to image
            Sheet.Hyperlinks.Add(imageCell,filename,[],[],['View Image ' num2str(imNum)]);
            % Create comment for image preview on mouse hover
            imageCell.AddComment;
            imageCell.Comment.Shape.Fill.UserPicture(filename);
            imageCell.Comment.Shape.Width = size(im,2)/2;
            imageCell.Comment.Shape.Height = size(im,1)/2;
        end
        % Delete the copied plots figure
        delete(fig);
        
        % Write Probe to G2 measurement results
        % Write row header
        rowHeaders2Cell = Sheet.get('Cells',dateCell.Row,numel(colHeaders)+3);
        rowHeaders2Cell.Value = rowHeaders2;
        rowHeaders2Cell.Font.Bold = 1;
        % Write Probe to G2 value
        rowNum = dateCell.Row;
        field = colHeaders2;
        val = tableData2{1};
        % Remove any html formatting
        val = regexprep(val, '<.*?>','');
        [~,fieldCol] = find(strcmp(xlData(1,:),field),1);
        if ~isempty(fieldCol)
            % Column exists, write the value in new row
            Sheet.get('Cells',rowNum,fieldCol).Value = val;
        end
        
        % Update number of rows
        numRows = Sheet.get('Cells').Find('*',Sheet.get('Cells',1,1),[],[],1,2).Row;
        
        % Autofit columns
        Sheet.UsedRange.Columns.AutoFit;
        
        % Create/modify chart
        if Sheet.ChartObjects.Count == 0
            % If no chart exists, create one
            chartShape = Sheet.Shapes.AddChart;
            chartShape.Select;
            Workbook.ActiveChart.ChartType = 'xlXYScatterLines';
            Workbook.ActiveChart.Axes(1).TickLabels.Orientation = 35;
        else
            % Select existing chart
            chartShape = Sheet.ChartObjects.Item(1);
            chartShape.Select;
        end
        % Set/update chart data
        coordsCol = Sheet.get('Range',Sheet.get('Cells',2,2),Sheet.get('Cells',numRows,2)).Value;
        coords = unique(coordsCol(cellfun(@ischar,coordsCol)));
        seriesCollection = Workbook.ActiveChart.SeriesCollection;
        for s = 1:numel(coords)
            if seriesCollection.Count < s
                % Create series
                seriesCollection.NewSeries;
            end
            series = seriesCollection.Item(s);
            series.Name = coords{s};
            % Get row numbers for this coord
            rowNums = find(strcmp(coordsCol,coords{s}));
            % X Data
            % Get test dates for this coord
            dateColumn = Sheet.get('Range',Sheet.get('Cells',2,1),Sheet.get('Cells',numRows,1));
            if dateColumn.Cells.Count == 1
                dateRow = 1;
            else
                dateRow = find(cellfun(@ischar,dateColumn.Value(1:rowNums(1))),1,'last');
            end
            dateRange = dateColumn.Cells.Item(dateRow);
            for n = 2:numel(rowNums)
                dateRow = find(cellfun(@ischar,dateColumn.Value(1:rowNums(n))),1,'last');
                dateRange = Excel.Union(dateRange,dateColumn.Cells.Item(dateRow));
            end
            series.XValues = dateRange;
            % Y Data
            valColumn = Sheet.get('Range',Sheet.get('Cells',2,3),Sheet.get('Cells',numRows,3));
            valRange = valColumn.Cells.Item(rowNums(1));
            for n = 2:numel(rowNums)
                valRange = Excel.Union(valRange,valColumn.Cells.Item(rowNums(n)));
            end
            series.Values = valRange;
        end
        Workbook.ActiveChart.HasTitle = 1;
        Workbook.ActiveChart.ChartTitle.Text = 'Grid Alignment';
        % Set/update chart position
        chartShape.Top = Sheet.get('Cells',numRows+2,1).Top;
        chartShape.Left = Sheet.get('Cells',numRows+2,1).Left+10;
        
        % Save the workbook
        invoke(Workbook, 'Save');
        msgbox('Export successful.');
    else
        % Don't have write access, file may be open in another program
        errordlg('Cannot export to excel file. The file may be open in another application.',...
            'Error');
    end
    % Close Excel
    invoke(Excel, 'Quit');
catch exception
    disp(getReport(exception));
    % Make sure to close excel if error occurs
    invoke(Excel, 'Quit');
end


% --- Executes during object creation, after setting all properties.
function gridAlignment_table_probeToG2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to gridAlignment_table_probeToG2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
data = cell(1,1);
set(hObject,'Data',data);
set(hObject,'RowName','Probe to G2');
set(hObject,'ColumnName','Distance (mm)');
set(hObject,'ColumnEditable',false(1,size(data,2)));


% --- Executes on button press in overlay_button_image1.
function overlay_button_image1_Callback(hObject, eventdata, handles)
% hObject    handle to overlay_button_image1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
[filename,pathname] = uigetfile({'*.bmp;*.jpg;*.tif;*.png;*.gif;*.dcm','All Image Files';...
        '*.*','All Files' },'Select Image(s)');

if filename ~= 0
    % Get the listbox that is also on this panel
    listbox = handles.overlay_listbox;
    % Display filenames in listbox
    files = get(listbox,'String');
    files{1} = filename;
    set(listbox,'String',files);
    % Set listbox 'Value' property
    set(listbox,'Value',1);
    
    % Show image
    axesHandle = handles.overlay_axes;
    set(axesHandle,'NextPlot','add');
    im = imread(fullfile(pathname,filename));
    handles.overlayImage1 = im;
    % Delete old image if exists
    if ishandle(handles.overlayImage1)
        delete(handles.overlayImage1);
    end
    
    % Get size of image
    s = size(handles.overlayImage1);
    imsize = [s(1) s(2)];
    % Create colour mask
    colour = handles.overlayImage1Colour;
    mask(:,:,1) = repmat(colour(1),imsize);
    mask(:,:,2) = repmat(colour(2),imsize);
    mask(:,:,3) = repmat(colour(3),imsize);
    mask = uint8(mask);
    
    handles.overlayImagePlot1 = imshow(mask.*im,'Parent',axesHandle);
    axis(axesHandle,'tight');
end
guidata(hObject,handles);


% --- Executes on button press in overlay_button_image2.
function overlay_button_image2_Callback(hObject, eventdata, handles)
% hObject    handle to overlay_button_image2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
[filename,pathname] = uigetfile({'*.bmp;*.jpg;*.tif;*.png;*.gif;*.dcm','All Image Files';...
        '*.*','All Files' },'Select Image(s)');

if filename ~= 0
    % Get the listbox that is also on this panel
    listbox = handles.overlay_listbox;
    % Display filenames in listbox
    files = get(listbox,'String');
    files{2} = filename;
    set(listbox,'String',files);
    % Set listbox 'Value' property
    set(listbox,'Value',2);
    
    % Show image
    axesHandle = handles.overlay_axes;
    set(axesHandle,'NextPlot','add');
    im = imread(fullfile(pathname,filename));
    handles.overlayImage2 = im;
    % Delete old image if exists
    if ishandle(handles.overlayImage2)
        delete(handles.overlayImage2);
    end
    
    % Get size of image
    s = size(handles.overlayImage2);
    imsize = [s(1) s(2)];
    % Create colour mask
    colour = handles.overlayImage2Colour;
    mask(:,:,1) = repmat(colour(1),imsize);
    mask(:,:,2) = repmat(colour(2),imsize);
    mask(:,:,3) = repmat(colour(3),imsize);
    mask = uint8(mask);
    
    handles.overlayImagePlot2 = imshow(mask.*im,'Parent',axesHandle);
    axis(axesHandle,'tight');
end
guidata(hObject,handles);

    
% --- Executes on slider movement.
function overlay_slider_transparency2_Callback(hObject, eventdata, handles)
% hObject    handle to overlay_slider_transparency2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
sliderVal = get(hObject,'Value');
if ishandle(handles.overlayImagePlot2)
    set(handles.overlayImagePlot2,'AlphaData',sliderVal);
end


% --- Executes during object creation, after setting all properties.
function overlay_slider_transparency2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to overlay_slider_transparency2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes during object creation, after setting all properties.
function overlay_slider_brightness2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to overlay_slider_brightness2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes during object creation, after setting all properties.
function overlay_slider_contrast2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to overlay_slider_contrast2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function overlay_slider_transparency1_Callback(hObject, eventdata, handles)
% hObject    handle to overlay_slider_transparency1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
sliderVal = get(hObject,'Value');
if ishandle(handles.overlayImagePlot1)
    set(handles.overlayImagePlot1,'AlphaData',sliderVal);
end

% --- Executes during object creation, after setting all properties.
function overlay_slider_transparency1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to overlay_slider_transparency1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes during object creation, after setting all properties.
function overlay_slider_brightness1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to overlay_slider_brightness1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on brightness/contrast slider movement and tint colour selected.
function overlay_updateImage_Callback(imageNum, handles)
% hObject    handle to overlay_slider_contrast1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
if ishandle(handles.(['overlayImagePlot' num2str(imageNum)]))
    % Get original image
    im = handles.(['overlayImage' num2str(imageNum)]);
    
    % Get size of image
    s = size(im);
    imsize = [s(1) s(2)];
    % Create colour mask
    colour = handles.(['overlayImage' num2str(imageNum) 'Colour']);
    mask(:,:,1) = repmat(colour(1),imsize);
    mask(:,:,2) = repmat(colour(2),imsize);
    mask(:,:,3) = repmat(colour(3),imsize);
    mask = uint8(mask);
    
    % Get factor to adjust brightness
    brightVal = get(handles.(['overlay_slider_brightness' num2str(imageNum)]),'Value');
    if brightVal > 0
        brightFactor = 1 + brightVal*5;
    else
        brightFactor = 1 + brightVal;
    end
    
    % Adjust contrast
    contrastVal = get(handles.(['overlay_slider_contrast' num2str(imageNum)]),'Value');
    if contrastVal > 0
        set(handles.(['overlayImagePlot' num2str(imageNum)]),'CData',...
            imadjust(mask.*(brightFactor*im),[0.5*contrastVal 1-0.5*contrastVal],[0 1]));
    else
        set(handles.(['overlayImagePlot' num2str(imageNum)]),'CData',...
            imadjust(mask.*(brightFactor*im),[0 1],[0.5*-contrastVal 1-0.5*-contrastVal]));
    end
end


% --- Executes during object creation, after setting all properties.
function overlay_slider_contrast1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to overlay_slider_contrast1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on button press in overlay_button_colourTint2.
function overlay_button_colourTint2_Callback(hObject, eventdata, handles)
% hObject    handle to overlay_button_colourTint2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Pick colour to tint image
colour = uisetcolor('Select Image 2 Tint Colour');
if numel(colour) == 3
    % Store colour in handles
    handles.overlayImage2Colour = colour;
    guidata(hObject,handles);
    % Update image
    overlay_updateImage_Callback(2, handles)
end


% --- Executes on button press in overlay_button_colourTint1.
function overlay_button_colourTint1_Callback(hObject, eventdata, handles)
% hObject    handle to overlay_button_colourTint1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Pick colour to tint image
colour = uisetcolor('Select Image 1 Tint Colour');
if numel(colour) == 3
    % Store colour in handles
    handles.overlayImage1Colour = colour;
    guidata(hObject,handles);
    % Update image
    overlay_updateImage_Callback(1, handles)
end
