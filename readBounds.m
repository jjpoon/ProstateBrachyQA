function [upper,lower] = readBounds(imageFile)
% READBOUNDS uses optical character recognition to detect the measurement
% labels on an ultrasound image.

im=imread(imageFile);

% Convert to gray scale
if size(im,3)==3 %RGB image
    im=rgb2gray(im);
end

% Read upper and lower bound
for i = 1:2
    
    % Crop image to label
    width = size(im,2);
    height = size(im,1);
    if i == 1
        % Upper bound
        imLabel=imcrop(im,[0.75*width 0.25*height 0.05*width 0.02*height]);
    elseif i == 2
        % Lower bound
        imLabel=imcrop(im,[0.75*width 0.77*height 0.05*width 0.02*height]);
    end
    
    % Enlarge image for better text recognition
    imLabel = imresize(imLabel,9);

    % Convert to BW
    threshold = graythresh(imLabel);
    imLabel = im2bw(imLabel,threshold);
    % Remove unwanted extrusions
    imLabel = imopen(imLabel,strel('disk',3));
    % Make black text on white background
    imLabel = imcomplement(imLabel);
%     figure;imshow(imLabel);
    
    % Save to file
    imwrite(imLabel,'text.bmp');
    % Text recognition using Tesseract-OCR, text outputted to out.txt
    system('tesseract text.bmp out scalelabels');
    % Read out.txt
    s = strtrim(fileread('out.txt'));
    % Find number, expression can begin with 0 or 1 '-', followed by any
    % number of digits separated by a decimal
    num = str2double(regexp(s,'-?\d*.\d*','match','once'));
    
    % Save as upper or lower bound
    if i == 1
        upper = num;
    elseif i == 2
        lower = num;
    end
    
end

% If upper or lower scale was not readable, assume image was taken in
% sagittal view and set scale readings to 6.4 and 0 cm.
if isnan(upper) || isnan(lower)
    upper = 6.4;
    lower = 0;
    
    % Set flag in handles for gui to show warning about assumed scale
    fig = findobj(get(0,'Children'),'Name','ProstateBrachyQA');
    handles = guidata(fig);
    
    handles.AssumedScale = 1;
    guidata(fig,handles);
end

end