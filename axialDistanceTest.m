function result = axialDistanceTest(imageFile)
% AXIALDISTANCETEST is for the axial distance measurement accuracy quality control test.
% The function compares the axial distance measurement with the known
% value and checks if the error is larger than 2 mm (absolute) or 2% (relative). 

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
    if strcmp(baselineVals{i,1},'Axial distance')
        baselineVal = baselineVals{i,2};
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

error = abs(newVal-baselineVal);
% Check measured axial distance measurement error
if (error > 2) || (error > 0.02*baselineVal)
    % Fail
    result = 0;
    disp('Axial distance measurement accuracy test: failed');
else
    % Pass
    result = 1;
    disp('Axial distance measurement accuracy test: passed');
end

end