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

% Get baseline value for this test
for i = 1:size(baselineFile,1)
    if strcmp(baselineFile{i,1},'Axial resolution')
        baselineVal = baselineFile{i,2};
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
newVal = sum(dist)/numel(dist);

disp(['Baseline value: ' num2str(baselineVal) ' mm']);
disp(['New value: ' num2str(newVal) ' mm']);

% Change in axial resolution (in mm)
change = abs(newVal - baselineVal);
% Check if max depth has changed by more than 1 cm
if change > 1
    % Fail
    result = 0;
    disp('Axial resolution test: failed');
else
    % Pass
    result = 1;
    disp('Axial resolution test: passed');
end

end