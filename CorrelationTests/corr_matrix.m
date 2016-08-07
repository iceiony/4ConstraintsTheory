function [fullCorr,matrixCorr] = corr_matrix(a,b)
    
rowCorr = zeros(1,length(a));
colCorr = zeros(length(a),1);

for i=1:length(a)
    rowCorr(i) = corr(a(i,:)',b(i,:)');
    colCorr(i) = corr(a(:,i),b(:,i));
end
    
rowCorr(isnan(rowCorr)) = 1;
colCorr(isnan(colCorr)) = 1;

fullCorr = rowCorr * colCorr / length(a);
matrixCorr = colCorr * rowCorr;