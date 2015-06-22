function result = lateralDistanceTest(imageFile)
% LATERALDISTANCETEST is for the lateral distance measurement accuracy quality control test.
% The function compares the lateral distance measurement with the known
% value and checks if the error is larger than 3 mm (absolute) or 3% (relative). 

% Get baseline values
if ~exist('Baseline.mat','file')
    % Read xls file if mat file not created yet
    baselineVals = readBaselineFile('Baseline.xls');
else
    % Get baseline value from mat file (faster)
    load('Baseline.mat');
end

% Get baseline value for this test
for i = 1:size(baselineVals,1)
    if strcmp(baselineVals{i,1},'Lateral distance')
        baselineVal = baselineVals{i,2};
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

error = abs(newVal-baselineVal);
% Check measured lateral distance measurement error
if (error > 3) || (error > 0.03*baselineVal)
    % Fail
    result = 0;
    disp('Lateral distance measurement accuracy test: failed');
else
    % Pass
    result = 1;
    disp('Lateral distance measurement accuracy test: passed');
end

end