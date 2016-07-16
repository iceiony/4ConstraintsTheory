function [cor,pval] = corr_multi_dim(x,y)
%Calculates correlations between two multi-dimensional random variables.
%The correlation matrix returned is the value of Pearson correlation on 
%each combination of columns of X and Y
cor = zeros(size(x,2),size(y,2));
pval = zeros(size(x,2),size(y,2));

for i=1:size(x,2)
    for j=1:size(y,2)
        [cor(i,j),pval(i,j)] = corr(x(:,i),y(:,j),'type','spearman');
    end
end
