img1 = extract_interest(imread('mpeg7/horse-1.gif'));
img2 = extract_interest(imread('mpeg7/horse-20.gif'));

sample1 = sample_points(img1,size(img1));
sample2 = sample_points(img2,size(img2));

class_count = 4;

subplot(1,2,1);
hold on;

idx = kmeans(sample1,class_count);
for i=1:class_count
    plot(sample1(idx==i,1),sample1(idx==i,2),'.');
end

subplot(1,2,2);
hold on;
idx = kmeans(sample2,class_count);
for i=1:class_count
    plot(sample2(idx==i,1),sample2(idx==i,2),'.');
end

