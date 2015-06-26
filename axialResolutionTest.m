function result = axialResolutionTest(imageFile)
% AXIALRESOLUTIONTEST is for the axial resolution quality control test.
% The function checks if the axial resolution has changed by
% more than 1 mm from the baseline value.

% Get baseline values
if ~exist('Baseline.mat','file')
    % Read xls file if mat file not created yet
    baselineFile = readBaselineFile('Baseline.xls');
else
    % Get baseline value from mat file (faster)
    load('Baseline.mat');
end

% Get baseline values for this test
for i = 1:size(baselineFile,1)
    if ~isempty(strfind(baselineFile{i,1},'Axial resolution'))
        baselineVals = [baselineFile{i,2:7}];
    end
end

% Load image and read labels
labels = readLabels(imageFile);
dist = [];
for n = 1:numel(labels)
    ind = strfind(labels{n},'Dist');
    % If label contains 'Dist'
    if ~isempty(ind)
        dist(n) = str2double(regexp(labels{n}(ind:end),'\d*\.\d*','match','once'));
    end
end
newVals = dist;

if numel(newVals)<4
    view = 'sagittal';
else
    view = 'axial';
end

if strcmp(view,'axial')
    % Display measurements for axial view
    disp('Baseline values (axial plane):');
    disp(['Proximal (left): ' sprintf('%.2f',baselineVals(1)) ' mm']);
    disp(['Proximal (right): ' sprintf('%.2f',baselineVals(2)) ' mm']);
    disp(['Distal (left): ' sprintf('%.2f',baselineVals(3)) ' mm']);
    disp(['Distal (right): ' sprintf('%.2f',baselineVals(4)) ' mm']);
    disp(' ');
    disp('New values (axial plane):');
    disp(['Proximal (left): ' sprintf('%.2f',newVals(1)) ' mm']);
    disp(['Proximal (right): ' sprintf('%.2f',newVals(2)) ' mm']);
    disp(['Distal (left): ' sprintf('%.2f',newVals(3)) ' mm']);
    disp(['Distal (right): ' sprintf('%.2f',newVals(4)) ' mm']);
    disp(' ');
else
    % Display measurements for sagittal view
    disp('Baseline values (longitudinal plane):');
    disp(['Proximal: ' sprintf('%.2f',baselineVals(5)) ' mm']);
    disp(['Distal: ' sprintf('%.2f',baselineVals(6)) ' mm']);
    disp(' ');
    disp('New values (longitudinal plane):');
    disp(['Proximal: ' sprintf('%.2f',newVals(1)) ' mm']);
    disp(['Distal: ' sprintf('%.2f',newVals(2)) ' mm']);
    disp(' ');
end

% Change in axial resolution (in mm)
if strcmp(view,'axial')
    change = abs(newVals - baselineVals(1:4));
else
    change = abs(newVals - baselineVals(5:6));
end
% Get results (0 or 1 if fail or pass requirement)
result = change<=1;
% Check if axial resolution has changed by more than 1 mm
if any(change > 1)
    % Fail
    disp('Axial resolution test: failed');
else
    % Pass
    disp('Axial resolution test: passed');
end

end