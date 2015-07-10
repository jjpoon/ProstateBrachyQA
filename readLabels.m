function labels = readLabels(imageFile)
% READLABELS uses optical character recognition to detect the measurement
% labels on an ultrasound image.

im=imageFile;
imagen=imcrop(im,[0 0.85*size(im,1) 0.5*size(im,2) 0.5*size(im,1)]);

% Convert to gray scale
if size(imagen,3)==3 %RGB image
    imagen=rgb2gray(imagen);
end

% Enlarge image
imagen = imresize(imagen,10);

% Find label boundaries
% Make labels white, everything else black
bwLabels = im2bw(imagen,0.02);
% Find position of labels
labelBoxes = regionprops(bwLabels,'BoundingBox');

labels = {};

% Read each label
for i = 1:numel(labelBoxes)
    
    % Crop image to label
    imLabel = imcrop(imagen,labelBoxes(i).BoundingBox);
    imLabel = imresize(imLabel,2);
    
    % Convert to BW
    threshold = graythresh(imLabel)+0.1;
    imLabel = im2bw(imLabel,threshold);
    % Make black text on white background
    imLabel = imcomplement(imLabel);
    
    % Save to file
    imwrite(imLabel,'text.bmp');
    % Text recognition using Tesseract-OCR, text outputted to out.txt
    system('tesseract text.bmp out');
    % Read out.txt and save as label output
    labels{i} = strtrim(fileread('out.txt'));
    
end

end