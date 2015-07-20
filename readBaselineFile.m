function baselineFile = readBaselineFile(fileName)
% READBASELINEFILE reads the xls file with the baseline values and stores
% them in a mat file for faster access. This function can be used to update
% the Baseline.mat file with new values entered into Baseline.xls

[num,txt,baselineFile] = xlsread(fileName);
save('Baseline.mat','baselineFile');
disp('Values from Baseline.xls successfully saved to Baseline.mat');

end

