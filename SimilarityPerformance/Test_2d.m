sample_size = [50,50];
img1 = extract_interest(imread('mpeg7/chicken-6.gif'));
img2 = extract_interest(imread('mpeg7/chicken-8.gif'));

sample_size(1) = min([sample_size(1),size(img1,1),size(img2,1)]);
sample_size(2) = min([sample_size(2),size(img1,2),size(img2,2)]);

sample1 = sample_points(img1,sample_size);
sample2 = sample_points(img2,sample_size);

[fullCorr,angleX,a,b] = corr_dim2D( sample1,sample2 );

figure();

% imagesc(sample1);
% subplot(1,2,2);
% imagesc(sample2);

subplot(1,2,1);
imagesc(a);
title(sprintf('Correlation: %f', fullCorr));

subplot(1,2,2);
imagesc(b);
title(sprintf('%0.2f', angleX * 180/pi));
