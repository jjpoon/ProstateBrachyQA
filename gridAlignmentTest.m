function result = gridAlignmentTest(varargin)
% GRIDALIGNMENTEST is for the needle template alignment quality control test.
% The function checks the difference between the actual needle location 
% (using the needle template) and the corresponding point on the electronic
% grid overlay. The four corners and the center of the grid are tested. 
% Alignment must be correct to within 3 mm.

% Load images and read labels
for i = 1:numel(varargin)
    imageFile = varargin{i};
    labels = readLabels(imageFile);
    for n = 1:numel(labels)
        ind = strfind(labels{n},'Dist');
        % If label contains 'Dist'
        if ~isempty(ind)
            dist = str2double(regexp(labels{n}(ind:end),'\d*\.\d*','match','once'));
            if ~isempty(dist)
                errors(i) = dist;
            end
        end
    end
end

disp(['Errors: ' num2str(errors)]);

% Compare errors between needle template and electronic grid
% Check if any errors (corners and center) are greater than 3 mm.
if any(errors>3)
    % Fail
    result = 0;
    disp('Needle template alignment test: failed');
else
    % Pass
    result = 1;
    disp('Needle template alignment test: passed');
end

end