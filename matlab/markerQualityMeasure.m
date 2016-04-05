%%
% Some experiments for quantifying the quality of the detected marker.
% This document is based on discussions with Mathias Nerup in
% april 2016.

% Locate bright or dark regions of a marker.
order = 5;
kernelsize = 31;
temp = generateSymmetryDetectorKernel(order, kernelsize);
tempDirection = generateSymmetryDetectorKernel(1, kernelsize);
figure(1)
threshold = 0.4 * max(abs(temp(:)));
phase = exp(0.1 * pi * 1i);
t1 = real(temp * phase.^order) > threshold;
t2 = real(temp * phase.^order) < -threshold;
imagesc(t1 - t2);

% Locate bright or dark regions of a marker.
% Handle the case where one of the black regions have been removed.
% Generate a mask for the location of the removed black region.
figure(2);
angleThreshold = pi / (2 * order);
t3 = angle(tempDirection * phase) < angleThreshold;
t4 = angle(tempDirection * phase) > -angleThreshold;
mask = 1 - 2 * (t3 & t4);
imagesc(mask);

% Use the mask to invert the expected color.
figure(3);
imagesc((t1 - t2) .* mask);