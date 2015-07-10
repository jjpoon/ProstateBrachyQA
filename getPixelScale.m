function ratio = getPixelScale(imageFile,varargin)
% GETPIXELSCALE calculates the mm/pixel ratio for a given ultrasound
% image. The upper and lower bound labels are read, and the top and bottom
% of the ultrasound image are found. The ratio is calculated using this
% information and can be used to convert from pixels to mm.

% If user has given the scale readings
if numel(varargin)>0
    upper = varargin{1};
    lower = varargin{2};
else
    % Read the upper and lower scale labels (in cm)
    [upper,lower] = readBounds(imageFile);
end
disp(['Upper bound: ' num2str(upper) ' cm']);
disp(['Lower bound: ' num2str(lower) ' cm']);

% Read image
im = imageFile;
width = size(im,2);
height = size(im,1);
% Crop to ultrasound image
% Initial crop to area of interest
im = imcrop(im,[0.15*width 0.2*height 0.7*width 0.65*height]);
% Make all non-black objects white
im = im2bw(im,0.01);
% Get indices of rows/columns with nonzero elements
[row col] = find(im);
% Crop tight to objects
im = im(min(row):max(row),min(col):max(col));

% Calculate pixel to mm ratio
% Height of ultrasound image in pixels
pixelLength = size(im,1);
% Height of ultrasound image in mm
mmLength = abs(upper - lower)*10;
% Pixel to mm conversion ratio
ratio = mmLength/pixelLength;

end