function result = areaTest(imageFile)
% AREATEST is for the area measurement accuracy quality control test.
% The function checks if the calculated area is within 5% of the actual 
% area value. 

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
    if strcmp(baselineVals{i,1},'Area')
        baselineVal = baselineVals{i,2};
    end
end

% Load image and read label
labels = readLabels(imageFile);
% Check if label contains 'Area'
ind = strfind(labels{1},'Area');
% If label contains 'Area'
if ~isempty(ind)
    % Find the first decimal number (the measurement) after 'Area'
    newVal = str2double(regexp(labels{1}(ind:end),'\d*\.\d*','match','once'));
end

disp(['Baseline value: ' num2str(baselineVal)]);
disp(['New value: ' num2str(newVal)]);

error = abs(newVal-baselineVal);
% Compare measured area and known area
if error > 0.05*baselineVal
    % Fail
    result = 0;
    disp('Area measurement accuracy test: failed');
else
    % Pass
    result = 1;
    disp('Area measurement accuracy test: passed');
end

end