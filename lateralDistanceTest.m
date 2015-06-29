function result = lateralDistanceTest(imageFile)
% LATERALDISTANCETEST is for the lateral distance measurement accuracy quality control test.
% The function compares the lateral distance measurement with the known
% value and checks if the error is larger than 3 mm (absolute) or 3% (relative). 

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
    if strcmp(baselineFile{i,1},'Lateral distance')
        knownVal = baselineFile{i,2};
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
        dist(n) = str2double(regexp(labels{n}(ind:end),'\d*\.\d*','match','once'));
    end
end
measuredVals = dist;

disp(['Known value: ' sprintf('%.2f',knownVal) ' mm']);

if numel(measuredVals) > 1
    disp(' ');
    disp('Measured values:');
    disp(['Proximal: ' sprintf('%.2f',measuredVals(1)) ' mm']);
    disp(['Distal: ' sprintf('%.2f',measuredVals(2)) ' mm']);
    disp(' ');
else
    disp(['Measured value: ' sprintf('%.2f',measuredVals)]);
end

if ~isempty(knownVal)
    error = abs(measuredVals-repmat(knownVal,size(measuredVals)));
else
    error = [];
end
% Check measured lateral distance measurement errors
result = error<=3 | error<=0.03*knownVal;
if isempty(result)
    disp('Missing information - could not complete test');
elseif any(result == 0)
    % Fail
    disp('Lateral distance measurement accuracy test: failed');
else
    % Pass
    disp('Lateral distance measurement accuracy test: passed');
end

end