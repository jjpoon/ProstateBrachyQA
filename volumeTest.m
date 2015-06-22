function result = volumeTest(imageFile)
% VOLUMETEST is for the volume measurement accuracy quality control test.
% The function checks if the calculated volume is within 5% of the actual 
% volume. 

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
    if strcmp(baselineVals{i,1},'Volume')
        baselineVal = baselineVals{i,2};
    end
end

% Load image and read label
labels = readLabels(imageFile);
% Check if label contains 'Pr-Vol'
ind = strfind(labels{1},'Pr-Vol');
% If label contains 'Pr-Vol'
if ~isempty(ind)
    % Find the first decimal number (the measurement) after 'Pr-Vol'
    newVal = str2double(regexp(labels{1}(ind:end),'\d*\.\d*','match','once'));
end

disp(['Baseline value: ' num2str(baselineVal) ' cm^3']);
disp(['New value: ' num2str(newVal) ' cm^3']);

error = abs(newVal-baselineVal);
% Compare measured volume and known volume
if error > 0.05*baselineVal
    % Fail
    result = 0;
    disp('Volume measurement accuracy test: failed');
else
    % Pass
    result = 1;
    disp('Volume measurement accuracy test: passed');
end

end