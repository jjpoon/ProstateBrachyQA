function result = axialDistanceTest(imageFile)
% AXIALDISTANCETEST is for the axial distance measurement accuracy quality control test.
% The function compares the axial distance measurement with the known
% value and checks if the error is larger than 2 mm (absolute) or 2% (relative). 

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
    if strcmp(baselineFile{i,1},'Axial distance')
        knownVal = baselineFile{i,2};
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
measuredVals = dist;

disp(['Known value: ' sprintf('%.2f',knownVal) ' mm']);

if numel(measuredVals) > 1
    disp(' ');
    disp('Measured values:');
    disp(['Left: ' sprintf('%.2f',measuredVals(1)) ' mm']);
    disp(['Right: ' sprintf('%.2f',measuredVals(2)) ' mm']);
    disp(' ');
else
    disp(['Measured value: ' sprintf('%.2f',measuredVals)]);
end

if ~isempty(knownVal)
    error = abs(measuredVals-repmat(knownVal,size(measuredVals)));
else
    error = [];
end
% Check measured axial distance measurement errors
result = error<=2 | error<=0.02*knownVal;
if isempty(result)
    disp('Missing information - could not complete test');
elseif any(result == 0)
    % Fail
    disp('Axial distance measurement accuracy test: failed');
else
    % Pass
    disp('Axial distance measurement accuracy test: passed');
end

end