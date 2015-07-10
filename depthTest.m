function result = depthTest(imageFile,varargin)
% DEPTHTEST is for the depth of penetration quality control test.
% The function checks if the maximum depth of pentration has changed by
% more than 1 cm from the baseline value.

% Input parser
p = inputParser;
addRequired(p,'imageFile',@isnumeric);
addParameter(p,'Plane',@ischar);
% Parse inputs
parse(p,imageFile,varargin{:});
plane = p.Results.Plane;

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
    if ~isempty(strfind(baselineFile{i,1},'Depth of penetration'))
        baselineVals = [baselineFile{i,2:3}];
    end
end
% Check what plane (axial or longitudinal) image was taken in and choose
% the corresponding baseline value
if strcmpi(plane,'longitudinal')
    baselineVal = baselineVals(2);
else
    baselineVal = baselineVals(1);
end

% Load image and read label
labels = readLabels(imageFile);
ind = strfind(labels{1},'Depth');
% If label contains 'Depth'
if ~isempty(ind)
    newVal = str2double(regexp(labels{1}(ind:end),'\d*\.\d*','match','once'));
end

disp(['Baseline value: ' num2str(baselineVal) ' mm']);
disp(['New value: ' num2str(newVal) ' mm']);

% Change in max depth (in cm)
change = abs(newVal - baselineVal)/10;
% Check if max depth has changed by more than 1 cm
if change > 1
    % Fail
    result = 0;
    disp('Depth of penetration test: failed');
else
    % Pass
    result = 1;
    disp('Depth of penetration test: passed');
end

end