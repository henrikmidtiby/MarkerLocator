function kernel = generateSymmetryDetectorKernel(order, kernelsize)

stepsize = 2 / (kernelsize-1);
temp1 = meshgrid(-1:stepsize:1);
kernel = temp1 + 1i*temp1';
id = abs(kernel) > 1;
%kernel(id) = 0;

magni = abs(kernel);
kernel = kernel.^order;
kernel = kernel.*exp(-8*magni.^2);
abs(kernel);

end