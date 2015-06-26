function result = lateralResolutionTest(imageFile)
% LATERALRESOLUTIONTEST is for the lateral resolution quality control test.
% The function checks if the lateral resolution has changed by
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
    if strcmp(baselineFile{i,1},'Lateral resolution')
        baselineVal = baselineFile{i,2};
    end
end

% Load image and read labels
labels = readLabels(imageFile);
dist = [];
for n = 1:numel(labels)
    % Check if label contains 'Dist'
    ind = strfind(labels{1},'Dist');
    % If label contains 'Dist'
    if ~isempty(ind)
        % Find the first decimal number (the measurement) after 'Dist'
        dist(n) = str2double(regexp(labels{1}(ind:end),'\d*\.\d*','match','once'));
    end
end
newVal = sum(dist)/numel(dist);

disp(['Baseline value: ' num2str(baselineVal) ' mm']);
disp(['New value: ' num2str(newVal) ' mm']);

% Change in lateral resolution (in mm)
change = abs(newVal - baselineVal);
% Check if max depth has changed by more than 1 cm
if change > 1
    % Fail
    result = 0;
    disp('Lateral resolution test: failed');
else
    % Pass
    result = 1;
    disp('Lateral resolution test: passed');
end

end