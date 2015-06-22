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

% Last Modified by GUIDE v2.5 19-Jun-2015 17:47:58

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
handles.upperScaleReading = cell(1,9);
handles.lowerScaleReading = cell(1,9);

% Set up tabs
warning off MATLAB:uitabgroup:OldVersion
% Create tab group
handles.tabgroup = uitabgroup(hObject,'Position',[0 0 1 1]);
% Create tabs
tab1 = uitab(handles.tabgroup,'Title','Grayscale');
tab2 = uitab(handles.tabgroup,'Title','Depth');
tab3 = uitab(handles.tabgroup,'Title','Axial Resolution');
tab4 = uitab(handles.tabgroup,'Title','Lateral Resolution');
tab5 = uitab(handles.tabgroup,'Title','Axial Distance');
tab6 = uitab(handles.tabgroup,'Title','Lateral Distance');
tab7 = uitab(handles.tabgroup,'Title','Area');
tab8 = uitab(handles.tabgroup,'Title','Volume');
tab9 = uitab(handles.tabgroup,'Title','Grid Alignment');
% Set tabs as parents of appropriate test panels
set(handles.grayscale_panel,'Parent',tab1);
set(handles.depth_panel,'Parent',tab2);
set(handles.axialResolution_panel,'Parent',tab3);
set(handles.lateralResolution_panel,'Parent',tab4);
set(handles.axialDistance_panel,'Parent',tab5);
set(handles.lateralDistance_panel,'Parent',tab6);
set(handles.area_panel,'Parent',tab7);

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


% --- Executes on button press in grayscale_button_images.
function button_images_Callback(hObject, eventdata, handles)
% hObject    handle to grayscale_button_images (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Open dialog for selecting image(s)
[filenames,pathname] = uigetfile({'*.bmp;*.jpg;*.tif;*.png;*.gif;*.dcm','All Image Files';...
    '*.*','All Files' },'Select Image(s)','MultiSelect','on');
if ischar(filenames)
    filenames = {filenames};
end
% If filenames is not 0 (0 if user pressed cancel)
if ~isnumeric(filenames)
    handles.imageFiles = fullfile(pathname,filenames);
    % Get the panel this button is on
    panel = get(hObject,'Parent');
    % Get the listbox that is also on this panel
    listbox = findobj(get(panel,'Children'),'Type','uicontrol','Style','listbox');
    % Set listbox 'Value' property
    set(listbox,'Value',numel(filenames));
    % Display filenames in listbox
    set(listbox,'String',filenames);
    % Enable Run Test button
    runTestButton = findobj(get(panel,'Children'),'Type','uicontrol','Style','pushbutton',...
        '-regexp','Tag','runTest');
    set(runTestButton,'Enable','on');
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


% --- Executes on button press in grayscale_button_runTest.
function button_runTest_Callback(hObject, eventdata, handles)
% hObject    handle to grayscale_button_runTest (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get test name (test function called and gui handles depend on this name)
testNum = get(handles.tabgroup,'SelectedIndex');
switch testNum
    case 1
        testName = 'grayscale';
    case 2
        testName = 'depth';
    case 3
        testName = 'axialResolution';
    case 4
        testName = 'lateralResolution';
    case 5
        testName = 'axialDistance';
    case 6
        testName = 'lateralDistance';
    case 7
        testName = 'area';
    case 8
        testName = 'volume';
    case 9
        testName = 'gridAlignment';
end
% Get function handle for correct test
testFunction = eval(['@' testName 'TestAuto']);

if isfield(handles,'imageFiles')
    % Get image filename
    imageFile = handles.imageFiles{1};
    
    % Run test, plot on given axes
    % Check if scale readings were set manually
    if ~isempty(handles.upperScaleReading{testNum}) && ~isempty(handles.lowerScaleReading{testNum})
        % Scale readings were inputted
        [result,baselineVal,newVal] = testFunction(imageFile,...
            'UpperScale',handles.upperScaleReading{testNum},'LowerScale',handles.lowerScaleReading{testNum},...
            'AxesHandle',handles.([testName,'_axes']));
    else
        % Read scale automatically from image
        [result,baselineVal,newVal] = testFunction(imageFile,'AxesHandle',handles.([testName,'_axes']));
    end
    % Units for labels depends on test type
    if strcmp(testName,'area')
        units = 'cm^2';
    elseif strcmp(testName,'volume')
        units = 'cm^3';
    else
        units = 'mm';
    end
    % Set baseline value label
    set(handles.([testName,'_text_baselineVal']),'String',sprintf('%.2f %s',baselineVal,units));
    % Set new value label
    set(handles.([testName,'_text_newVal']),'String',sprintf('%.2f %s',newVal,units));
    % Set test result label
    if result == 1
        set(handles.([testName,'_text_result']),'ForegroundColor',[0 0.75 0],'String','PASS');
    else
        set(handles.([testName,'_text_result']),'ForegroundColor',[1 0 0],'String','FAIL');
    end
end


% --- Executes on button press in grayscale_button_setScale.
function button_setScale_Callback(hObject, eventdata, handles)
% hObject    handle to grayscale_button_setScale (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get test number
testNum = get(handles.tabgroup,'SelectedIndex');

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
