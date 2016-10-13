function [ out_img ] = extract_interest( img )
%Extracts rectangle of interest in a image 
[n,m] = size(img);

idx = find(img >= 1,1);
minX = floor(idx/n) + 1;

idx = find(img >= 1,1,'last');
maxX = floor(idx/n) + 1;

idx = find(img' >= 1,1);
minY =   floor(idx/m) + 1;

idx = find(img' >= 1,1,'last');
maxY =   floor(idx/m) + 1;

out_img = img(minY:maxY,minX:maxX);

end

