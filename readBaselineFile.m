function baselineFile = readBaselineFile(fileName)
% READBASELINEFILE reads the xls file with the baseline values.

[num,txt,baselineFile] = xlsread(fileName);
disp('Values from Baseline.xls successfully read');

end

