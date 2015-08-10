function freq = readFrequency(imageFile)
% READFREQUENCY uses optical character recognition to read the frequency
% label on the image.

im=imageFile;

% Convert to gray scale
if size(im,3)==3 %RGB image
    im=rgb2gray(im);
end

% Crop image to label
width = size(im,2);
height = size(im,1);
imLabel=imcrop(im,[0.88*width 0.775*height 0.2*width 0.035*height]);

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
system('tesseract text.bmp out');
% Read out.txt
s = strtrim(fileread('out.txt'));
% Find number followed by MHz
freq = str2double(regexpi(s,'\d*(?= MHz)','match','once'));

end