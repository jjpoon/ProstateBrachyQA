function result = grayscaleTest(imageFile)
% GRAYSCALETEST is for the grayscale visibility quality control test.
% The function checks if the length of the gradient strip has changed by
% more than 10% from the baseline measurement.

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
    if ~isempty(strfind(baselineFile{i,1},'Grayscale'))
        baselineVal = baselineFile{i,2};
    end
end

% Load image and read label
labels = readLabels(imageFile);
% Check if label contains 'Dist'
ind = strfind(labels{1},'Dist');
% If label contains 'Dist'
if ~isempty(ind)
    % Find the first decimal number (the measurement) after 'Dist'
    newVal = str2double(regexp(labels{1}(ind:end),'\d*\.\d*','match','once'));
end

disp(['Baseline value: ' num2str(baselineVal) ' mm']);
disp(['New value: ' num2str(newVal) ' mm']);

% Change in length of gradient strip
change = abs(newVal - baselineVal);
% Check if length has changed by more than 10%
if change > 0.1*baselineVal
    % Fail
    result = 0;
    disp('Grayscale visbility test: failed');
else
    % Pass
    result = 1;
    disp('Grayscale visbility test: passed');
end

end