Make;

imgRgb = imread('img_rgb.ppm');
imgDepth = imread('img_depth.pgm');
imgMask = imgDepth == 0 | imgDepth == 255;

spaceSigmas = [12 5 8];
rangeSigmas = [0.2 0.08 0.02];

for ii = 1 : 10
tic;
imgDepthBf = mex_cross_bilateral_filter(imgDepth, rgb2gray(imgRgb), imgMask, spaceSigmas, rangeSigmas);
tElapsed = toc;
fprintf('Time elapsed: %f\n', tElapsed);
end

%%

mask = false(480, 640);
mask(45:470, 36:600) = 1;
H = 470-45+1;
W = 600-36+1;

figure;
subplot(1,2,1); imagesc(reshape(imgDepth(mask), [H, W])); colormap('gray');
subplot(1,2,2); imagesc(reshape(imgDepthBf(mask), [H, W])); colormap('gray');

