% Script for removing circle outlines from all ultrasound images in folder
folderName = 'No Outline Images';
d = dir(fullfile(folderName, '/*.bmp'));

for n = 1:numel(d)
    imageFile = d(n).name;
    im = imread(imageFile);
    im_outline = im2bw(im,0.99);
    mask = imcomplement(im_outline);
    im_black_outline = rgb2gray(im).*uint8(mask);
    im_close = imclose(im_black_outline,strel('disk',1));
    im(300:end-300,200:end-200,1) = im_close(300:end-300,200:end-200);
    im(300:end-300,200:end-200,2) = im_close(300:end-300,200:end-200);
    im(300:end-300,200:end-200,3) = im_close(300:end-300,200:end-200);
    imshow(im);
    imwrite(im,fullfile(folderName,imageFile));
end