function result = planningComputerTest(imageFile)
% PLANNINGCOMPUTERTEST is for the treatment planning computer quality control test.
% The function checks if the volumes calculated by the ultrasound system
% and the treatment planning computer agree to within 5%.

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
    if strcmp(baselineVals{i,1},'Planning computer')
        compVolume = baselineVals{i,2};
    end
end

% Load image and read label
labels = readLabels(imageFile);
% Check if label contains 'Pr-Vol'
ind = strfind(labels{1},'Pr-Vol');
% If label contains 'Pr-Vol'
if ~isempty(ind)
    % Find the first decimal number (the measurement) after 'Pr-Vol'
    usVolume = str2double(regexp(labels{1}(ind:end),'\d*\.\d*','match','once'));
end

disp(['Baseline value: ' num2str(compVolume)]);
disp(['New value: ' num2str(usVolume)]);

error = abs(usVolume - compVolume);
average = (usVolume + compVolume)/2;
% Check if volumes agree to within 5%
if error/average > 0.05
    % Fail
    result = 0;
    disp('Treatment planning computer test: failed');
else
    % Pass
    result = 1;
    disp('Treatment planning computer test: passed');
end

end