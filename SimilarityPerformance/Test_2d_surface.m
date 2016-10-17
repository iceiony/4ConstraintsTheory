images = dir('./mpeg7/*.gif');
shapeNr = length(images);

disp('Reading image data')
%read all images with selected part of interest
for i = 1:shapeNr     
    images(i).data = extract_interest(imread(sprintf('mpeg7/%s',images(i).name)));
end

disp('Measuring similarity');
similarities = zeros(shapeNr,shapeNr);
sample_size = [70,70];

tic
for i = 1:shapeNr
    disp(sprintf('\n\r%%%0.3f',100*i/shapeNr));
    for j = i:shapeNr
        if mod(j,140) == 0
            fprintf('.')
        end
        img1 = images(i).data;
        img2 = images(j).data;

        current_size = sample_size;
        current_size(1) = min([sample_size(1),size(img1,1),size(img2,1)]);
        current_size(2) = min([sample_size(2),size(img1,2),size(img2,2)]);

        sample1 = sample_points(img1,current_size);
        sample2 = sample_points(img2,current_size);

        [correlation,~,~,~] = corr_dim2D( sample1,sample2 );
        similarities(i,j) = correlation;
    end
end
toc

bullsEyeCount = 0;
records = [];
for i = 1:shapeNr
    category = ceil(i / 20);
    similarityMeasures = [similarities(1:i-1,i)' similarities(i,i:end)]; 
    sorted = sort(similarityMeasures,'descend');
    [~,topIdx,~] = intersect(similarityMeasures,sorted(1:40));
    similarCategories = ceil(topIdx ./ 20);
    
    matchedCount = sum(similarCategories == category);
    
    records(end+1) = matchedCount;
    bullsEyeCount = bullsEyeCount + matchedCount;
end
plot(records);
disp(sprintf('Bull''s eye score %%%0.2f', bullsEyeCount*100 / (20*1400)));
