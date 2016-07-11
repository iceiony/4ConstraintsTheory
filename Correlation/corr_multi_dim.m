function [cor] = corr_multi_dim(x,y)
%Calculates correlations between two multi-dimensional random variables.
%The correlation matrix returned is the value of Pearson correlation on 
%each combination of columns of X and Y
cor = zeros(size(x,2),size(y,2));

for i=1:size(x,2)
    for j=1:size(y,2)
        corMatrix = corrcoef(x(:,i),y(:,j));
        cor(i,j) = corMatrix(2);
    end
end
