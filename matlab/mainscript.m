%% Load image
%img = double(imread('edges.png'));
img = double(imread('patterns.png'));
img = imresize(img, 0.5);

%% Make kernel
kernel = generateSymmetryDetectorKernel(5, 50);


%% Apply kernel
res = conv2(img(:, :, 1), kernel);
%figure(1); imagesc(img(:, :, 1)); colormap(gray); axis equal
figure(2); imagesc((abs(res))); colormap(gray); axis equal
%figure(3); imagesc(angle(res));

%% Threshold results
ordered = sort(abs(res(:)));
thres = ordered(ceil(0.9998*length(ordered)));
figure(4); imagesc(abs(res) > thres); colormap(gray); axis equal;
